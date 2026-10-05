/**
MIT License

Copyright (c) 2023 Xin Zheng <xinzheng@zju.edu.cn>.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <trajlo/core/odometry.h>
#include <trajlo/utils/sophus_utils.hpp>

#include <fstream>
#include <iomanip>

uint64_t getCurrTime() {
  return std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::high_resolution_clock::now().time_since_epoch())
      .count();
}

namespace traj {
TrajLOdometry::TrajLOdometry(const TrajConfig& config)
    : config_(config), isFinish(false) {
  min_range_2_ = config.min_range * config.min_range;
  converge_thresh_ = config.converge_thresh;

  laser_data_queue.set_capacity(100);
  imu_data_queue.set_capacity(5000);
  imu_stream_en_ = config.imu_en && !config.imu_topic.empty();
  // T_body_lidar follows the harness convention p_imu = R * p_lidar + T,
  // i.e. it IS T_imu_lidar; state/body frame stays the lidar frame
  R_l_i_ = config.T_body_lidar.so3().inverse();
  t_i_l_ = config.T_body_lidar.translation();
  imu_time_offset_ns_ = (int64_t)(config.time_offset * 1e9);

  init_interval_ = config.init_interval;
  window_interval_ = config.seg_interval;
  max_frames_ = config.seg_num;

  map_.reset(new MapManager(config.ds_size, config.voxel_size,
                            config.planer_thresh, config.max_voxel_num,
                            config.max_range));
  if (config.map_type == "kdtree") {
    map_->UseKdTree(config.kd_min_dist, config.kd_max_nn_dist);
  } else if (config.map_type != "voxel") {
    std::cerr << "unknown mapping.map_type '" << config.map_type
              << "', using voxel" << std::endl;
  }

  // setup marginalization
  marg_H.setZero(POSE_SIZE, POSE_SIZE);
  marg_b.setZero(POSE_SIZE);
  double init_pose_weight = config.init_pose_weight;
  marg_H.diagonal().setConstant(init_pose_weight);
}

void TrajLOdometry::Start() {
  auto lo_func = [&] {
    int frame_id = 0;
    Scan::Ptr curr_scan;
    bool first_scan = true;
    Measurement::Ptr measure;

    while (true) {
      /*
       * this thread will block until the valid scan coming
       * */
      laser_data_queue.pop(curr_scan);
      if (!curr_scan.get()) break;

      if (first_scan) {
        last_begin_t_ns_ = curr_scan->timestamp;
        last_end_t_ns_ = last_begin_t_ns_ + init_interval_;
        first_scan = false;
      }
      PointCloudSegment(curr_scan, measure);

      while (!measure_cache.empty()) {
        measure = measure_cache.front();
        measure_cache.pop_front();

        // 1. range filter & compute the relative timestamps
        std::vector<Eigen::Vector4d> points;
        RangeFilter(measure, points);

        const auto& tp = measure->tp;

        if (imu_stream_en_) {
          // cover this segment with imu samples; keep enough history for the
          // oldest window segment (retro preintegration at init flip) plus
          // one sample at/before its begin for boundary interpolation
          FetchImuUntil(tp.second);
          const int64_t trim_t = measurements.empty()
                                     ? tp.first
                                     : measurements.begin()->first.first;
          while (imu_buffer_.size() >= 2 && imu_buffer_[1]->t_ns <= trim_t) {
            imu_buffer_.pop_front();
          }
          if (frame_id % 500 == 0) {
            size_t in_seg = 0;
            for (const auto& d : imu_buffer_) {
              if (d->t_ns > tp.first && d->t_ns <= tp.second) in_seg++;
            }
            std::cout << "[imu] seg " << frame_id << " samples in segment: "
                      << in_seg << " buffered: " << imu_buffer_.size()
                      << " total: " << imu_total_count_ << std::endl;
          }

          if (!imu_opt_active_) TryFinishImuInit(tp);
        }

        if (!map_->IsInit()) {
          T_wc_curr = Sophus::SE3d();
          map_->MapInit(points);

          // standing start (vel/bias zero; bias is overwritten by the static
          // initializer once IMU support is active)
          frame_states_[tp.second] = PoseVelBiasStateWithLin<double>(
              tp.second, T_wc_curr, Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), true);
          trajectory_.emplace_back(tp.first, T_wc_curr);
          map_->SetInit();

          T_prior = Sophus::SE3d();
        } else {
          measure->pseudoPrior = T_prior;
          measurements[tp] = measure;
          if (imu_opt_active_) measure->preint = BuildSegmentPreint(tp);

          Sophus::SE3d T_w_pred;
          if (measure->preint) {
            // imu prediction supplies pose AND velocity for the new state
            const auto s_pred = measure->preint->predict(
                frame_states_[tp.first].getState(), g_w_, tp.second);
            T_w_pred = s_pred.T_w_i;
            frame_states_[tp.second] = PoseVelBiasStateWithLin<double>(
                tp.second, s_pred.T_w_i, s_pred.vel_w_i, s_pred.bias_gyro,
                s_pred.bias_accel, false);
          } else {
            T_w_pred = frame_states_[tp.first].getPose() * T_prior;
            const auto& prev_state = frame_states_[tp.first].getState();
            frame_states_[tp.second] = PoseVelBiasStateWithLin<double>(
                tp.second, T_w_pred, prev_state.vel_w_i, prev_state.bias_gyro,
                prev_state.bias_accel, false);
          }

          // 2. preprocess the point cloud.
          map_->PreProcess(points, tp);

          if (!isMove_) {
            isMove_ = (T_w_pred).translation().norm() > 0.5;
          } else {
            map_->ComputeThreshold();
          }

          // 3. find the optimal control poses based on the geometric and motion
          // constrains
          Optimize();

          T_wc_curr = frame_states_[tp.second].getPose();
          Sophus::SE3d model_deviation = T_w_pred.inverse() * T_wc_curr;
          map_->UpdateModelDeviation(model_deviation);
          T_prior = frame_states_[tp.first].getPose().inverse() *
                    frame_states_[tp.second].getPose();

          // 4. marginalize the oldest segment and update the map using points
          // belond to the oldest segment.
          Marginalize();

          // map & trajectory visualization
          if (vis_data_queue &&
              ((frame_id / config_.seg_num) % (config_.frame_num) == 0)) {
            ScanVisData::Ptr visData(new ScanVisData);

            posePair pp{frame_states_[tp.first].getPose(),
                        frame_states_[tp.second].getPose()};
            UndistortRawPoints(measure->points, visData->data, pp);

            visData->T_w = config_.T_vis_lidar*pp.first;
            vis_data_queue->push(visData);  // may block the thread
          }
        }
        frame_id++;
      }
    }

    if (vis_data_queue) vis_data_queue->push(nullptr);

    // save pose in window
    for (const auto& kv : frame_states_) {
      trajectory_.emplace_back(kv.first, kv.second.getPose());
    }

    // Here, you can save the trajectory for comparison
    if(config_.save_pose){
      std::cout << "Start Pose Saving!" << std::endl;
      std::ofstream os(config_.pose_file_path);
      os << "# timestamp tx ty tz qx qy qz qw" << std::endl;

      for(const auto& p:trajectory_){
        Sophus::SE3d pose_body=config_.T_body_lidar*p.second*config_.T_body_lidar.inverse();
        Sophus::SE3d pose_gt=pose_body*config_.T_body_gt;

        os << std::scientific << std::setprecision(18)
           << p.first * 1e-9 << " " << pose_gt.translation().x()
           << " " << pose_gt.translation().y() << " "
           << pose_gt.translation().z() << " "
           << pose_gt.so3().unit_quaternion().x() << " "
           << pose_gt.so3().unit_quaternion().y() << " "
           << pose_gt.so3().unit_quaternion().z() << " "
           << pose_gt.so3().unit_quaternion().w() << std::endl;
      }
      os.close();
      std::cout << "Finish Pose Saving!" << std::endl;
    }

    isFinish = true;
    std::cout << "Finisher LiDAR Odometry " << std::endl;
  };

  processing_thread_.reset(new std::thread(lo_func));
}

/*
 * Blocking-drain the imu queue until the buffer covers t_ns (or the stream
 * ends). A watchdog aborts with a clear error instead of deadlocking the
 * harness when imu_en is set but the bag never delivers the imu topic.
 * */
void TrajLOdometry::FetchImuUntil(int64_t t_ns) {
  int idle_ms = 0;
  while (!imu_stream_ended_ &&
         (imu_buffer_.empty() || imu_buffer_.back()->t_ns < t_ns)) {
    ImuData::Ptr data;
    if (imu_data_queue.try_pop(data)) {
      idle_ms = 0;
      if (!data.get()) {
        imu_stream_ended_ = true;
        break;
      }
      // express the sample in the lidar (state) frame; centripetal lever-arm
      // correction included, angular-acceleration term omitted
      ImuData::Ptr d(new ImuData);
      d->t_ns = data->t_ns + imu_time_offset_ns_;
      d->gyr = R_l_i_ * data->gyr;
      d->acc =
          R_l_i_ * (data->acc + data->gyr.cross(data->gyr.cross(t_i_l_)));
      imu_buffer_.push_back(d);
      imu_total_count_++;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (++idle_ms > 10000) {
        std::cerr << "[imu] FATAL: no IMU data within 10s while imu is "
                     "enabled. Wrong imu topic?"
                  << std::endl;
        std::exit(3);
      }
    }
  }
}

/*
 * Linear interpolation of the imu stream at an arbitrary time (virtual
 * boundary sample for segment-aligned preintegration).
 * */
ImuData TrajLOdometry::InterpImuAt(int64_t t_ns) const {
  size_t hi = 0;
  while (hi < imu_buffer_.size() && imu_buffer_[hi]->t_ns < t_ns) hi++;
  if (hi == 0) {
    ImuData d = *imu_buffer_.front();
    d.t_ns = t_ns;
    return d;
  }
  if (hi == imu_buffer_.size()) {
    ImuData d = *imu_buffer_.back();
    d.t_ns = t_ns;
    return d;
  }
  const auto& a = *imu_buffer_[hi - 1];
  const auto& b = *imu_buffer_[hi];
  const double alpha =
      b.t_ns == a.t_ns ? 0.0
                       : (double)(t_ns - a.t_ns) / (double)(b.t_ns - a.t_ns);
  ImuData d;
  d.t_ns = t_ns;
  d.gyr = (1 - alpha) * a.gyr + alpha * b.gyr;
  d.acc = (1 - alpha) * a.acc + alpha * b.acc;
  return d;
}

/*
 * Preintegrate the imu stream over one segment [tp.first, tp.second],
 * with virtual samples interpolated exactly at both boundaries. Returns
 * nullptr when the stream does not cover the segment (then the caller
 * falls back to the pseudoPrior motion constraint).
 * */
ImuPreintegration::Ptr TrajLOdometry::BuildSegmentPreint(const tStampPair& tp) {
  if (imu_buffer_.empty() || imu_buffer_.back()->t_ns < tp.second ||
      imu_buffer_.front()->t_ns > tp.first) {
    return nullptr;
  }
  const auto& s0 = frame_states_.at(tp.first).getState();
  ImuPreintegration::Ptr pre(new ImuPreintegration(
      s0.bias_gyro, s0.bias_accel, config_.sigma_ng, config_.sigma_na));
  pre->integrate(InterpImuAt(tp.first));
  for (const auto& d : imu_buffer_) {
    if (d->t_ns > tp.first && d->t_ns < tp.second) pre->integrate(*d);
  }
  pre->integrate(InterpImuAt(tp.second));
  return pre;
}

/*
 * Feed the stand-still initializer; once enough span is collected, align
 * gravity, seed the gyro bias, rebuild the window states with vel/bias,
 * widen the marginalization prior to 15x15 and activate imu factors.
 * */
void TrajLOdometry::TryFinishImuInit(const tStampPair& tp) {
  for (const auto& d : imu_buffer_) {
    if (d->t_ns > init_fed_until_ && d->t_ns <= tp.second) {
      static_init_.add(*d);
      init_fed_until_ = d->t_ns;
    }
  }
  if (!static_init_.ready(config_.imu_init_time)) return;
  if (!map_->IsInit() || frame_states_.empty()) return;

  g_w_ = static_init_.gravity();
  const Eigen::Vector3d bg = static_init_.gyro_bias();
  const Eigen::Vector3d ba = Eigen::Vector3d::Zero();
  if (static_init_.gyro_std().maxCoeff() > 0.05) {
    std::cout << "[imu] WARNING: gyro std "
              << static_init_.gyro_std().transpose()
              << " during init - platform may not have been static"
              << std::endl;
  }

  // rebuild window states: bias seeded, velocity from pose differences
  Sophus::SE3d prev_pose;
  int64_t prev_t = 0;
  bool first = true;
  for (auto& kv : frame_states_) {
    const Sophus::SE3d pose = kv.second.getPose();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    if (!first && kv.first > prev_t) {
      vel = (pose.translation() - prev_pose.translation()) /
            ((kv.first - prev_t) * 1e-9);
    }
    kv.second = PoseVelBiasStateWithLin<double>(kv.first, pose, vel, bg, ba,
                                                kv.second.isLinearized());
    prev_pose = pose;
    prev_t = kv.first;
    first = false;
  }

  // widen the marginalization prior: keep the accumulated pose block, seed
  // diagonal info for vel/bg/ba of the linearized state
  Eigen::MatrixXd H_old = marg_H;
  Eigen::VectorXd b_old = marg_b;
  marg_H = Eigen::MatrixXd::Zero(POSE_VEL_BIAS_SIZE, POSE_VEL_BIAS_SIZE);
  marg_b = Eigen::VectorXd::Zero(POSE_VEL_BIAS_SIZE);
  marg_H.topLeftCorner(POSE_SIZE, POSE_SIZE) = H_old;
  marg_b.head(POSE_SIZE) = b_old;
  marg_H.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * 1e4;   // vel
  marg_H.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * 1e4;   // bg
  marg_H.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 1e2; // ba

  imu_opt_active_ = true;

  // retroactively attach imu factors to segments already in the window;
  // without them the vel/bias columns of interior states would be
  // unconstrained and the first solve would be singular
  size_t retro = 0;
  for (auto& m : measurements) {
    if (!m.second->preint) {
      m.second->preint = BuildSegmentPreint(m.first);
      if (m.second->preint) retro++;
    }
  }

  std::cout << "[imu] static init done: span " << static_init_.span()
            << "s, samples " << static_init_.count() << ", g_w "
            << g_w_.transpose() << ", bg " << bg.transpose()
            << ", retro-preint " << retro << "/" << measurements.size()
            << std::endl;
}

/*
 * The analytic Jacobians in the paper are derived in SE(3) form. For the
 * efficiency in our implementation, we instead update poses in SO(3)+R3
 * form. The connection between them has been discussed in
 * https://gitlab.com/VladyslavUsenko/basalt/-/issues/37
 * */
void TrajLOdometry::Optimize() {
  // per-state block width: pose only in LO mode, pose+vel+bias in LIO mode
  const size_t S = imu_opt_active_ ? POSE_VEL_BIAS_SIZE : POSE_SIZE;

  AbsOrderMap aom;
  for (const auto& kv : frame_states_) {
    aom.abs_order_map[kv.first] = std::make_pair(aom.total_size, S);
    aom.total_size += S;
    aom.items++;
  }

  Eigen::MatrixXd abs_H;
  Eigen::VectorXd abs_b;

  for (int iter = 0; iter < config_.max_iterations; iter++) {
    abs_H.setZero(aom.total_size, aom.total_size);
    abs_b.setZero(aom.total_size);

    // 两帧优化
    for (auto& m : measurements) {
      int64_t idx_prev = m.first.first;
      int64_t idx_curr = m.first.second;

      const auto& prev = frame_states_[idx_prev];
      const auto& curr = frame_states_[idx_curr];

      posePair pp{prev.getPose(), curr.getPose()};
      const tStampPair& tp = m.second->tp;  //{idx_prev,idx_curr};

      // 1. Geometric constrains from lidar point cloud.
      map_->PointRegistrationNormal({prev, curr}, tp, m.second->delta_H,
                                    m.second->delta_b, m.second->lastError,
                                    m.second->lastInliers);

      // assemble the full two-endpoint linearized system of this segment
      // (registration + motion factor + bias walk) in seg_H/seg_b; kept on
      // the measurement so Marginalize can reuse the converged linearization
      auto& seg_H = m.second->seg_H;
      auto& seg_b = m.second->seg_b;
      seg_H.setZero(2 * S, 2 * S);
      seg_b.setZero(2 * S);

      {  // registration: pose part of each endpoint
        const auto& dH = m.second->delta_H;
        const auto& db = m.second->delta_b;
        seg_H.block<POSE_SIZE, POSE_SIZE>(0, 0) +=
            dH.topLeftCorner<POSE_SIZE, POSE_SIZE>();
        seg_H.block<POSE_SIZE, POSE_SIZE>(0, S) +=
            dH.topRightCorner<POSE_SIZE, POSE_SIZE>();
        seg_H.block<POSE_SIZE, POSE_SIZE>(S, 0) +=
            dH.bottomLeftCorner<POSE_SIZE, POSE_SIZE>();
        seg_H.block<POSE_SIZE, POSE_SIZE>(S, S) +=
            dH.bottomRightCorner<POSE_SIZE, POSE_SIZE>();
        seg_b.segment<POSE_SIZE>(0) += db.head<POSE_SIZE>();
        seg_b.segment<POSE_SIZE>(S) += db.tail<POSE_SIZE>();
      }

      if (imu_opt_active_ && m.second->preint) {
        // 2a. IMU preintegration factor (replaces the pseudoPrior motion
        // constraint) + bias random walk between the endpoints
        const auto& pre = *m.second->preint;
        const auto& s0_cur = prev.getState();
        const auto& s1_cur = curr.getState();
        const auto& s0_J = prev.isLinearized() ? prev.getStateLin() : s0_cur;
        const auto& s1_J = curr.isLinearized() ? curr.getStateLin() : s1_cur;

        Eigen::Matrix<double, 9, 1> r, r_lin;
        Eigen::Matrix<double, 9, 15> J0, J1;
        pre.evaluate(s0_J, s1_J, g_w_, r_lin, &J0, &J1);
        pre.evaluate(s0_cur, s1_cur, g_w_, r);

        const Eigen::Matrix<double, 9, 9> W =
            pre.information() * config_.imu_weight;
        seg_H.block(0, 0, 15, 15) += J0.transpose() * W * J0;
        seg_H.block(0, S, 15, 15) += J0.transpose() * W * J1;
        seg_H.block(S, 0, 15, 15) += J1.transpose() * W * J0;
        seg_H.block(S, S, 15, 15) += J1.transpose() * W * J1;
        seg_b.segment(0, 15) -= J0.transpose() * W * r;
        seg_b.segment(S, 15) -= J1.transpose() * W * r;

        const double dt = std::max(pre.dt_total(), 1e-4);
        const double w_bg = 1.0 / (config_.sigma_bg_rw * config_.sigma_bg_rw * dt);
        const double w_ba = 1.0 / (config_.sigma_ba_rw * config_.sigma_ba_rw * dt);
        const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        const Eigen::Vector3d r_bg = s1_cur.bias_gyro - s0_cur.bias_gyro;
        const Eigen::Vector3d r_ba = s1_cur.bias_accel - s0_cur.bias_accel;
        seg_H.block<3, 3>(9, 9) += w_bg * I3;
        seg_H.block<3, 3>(S + 9, S + 9) += w_bg * I3;
        seg_H.block<3, 3>(9, S + 9) -= w_bg * I3;
        seg_H.block<3, 3>(S + 9, 9) -= w_bg * I3;
        seg_b.segment<3>(9) += w_bg * r_bg;
        seg_b.segment<3>(S + 9) -= w_bg * r_bg;
        seg_H.block<3, 3>(12, 12) += w_ba * I3;
        seg_H.block<3, 3>(S + 12, S + 12) += w_ba * I3;
        seg_H.block<3, 3>(12, S + 12) -= w_ba * I3;
        seg_H.block<3, 3>(S + 12, 12) -= w_ba * I3;
        seg_b.segment<3>(12) += w_ba * r_ba;
        seg_b.segment<3>(S + 12) -= w_ba * r_ba;
      } else {
        // 2b. Motion constrains behind continuous movement.
        // Log(Tbe)-Log(prior) Equ.(6)
        Sophus::SE3d T_be = pp.first.inverse() * pp.second;
        Sophus::Vector6d tau = Sophus::se3_logd(T_be);
        Sophus::Vector6d res = tau - Sophus::se3_logd(m.second->pseudoPrior);

        Sophus::Matrix6d J_T_w_b;
        Sophus::Matrix6d J_T_w_e;
        Sophus::Matrix6d rr_b;
        Sophus::Matrix6d rr_e;

        if (prev.isLinearized() || curr.isLinearized()) {
          pp = std::make_pair(prev.getPoseLin(), curr.getPoseLin());
          T_be = pp.first.inverse() * pp.second;
          tau = Sophus::se3_logd(T_be);
        }

        Sophus::rightJacobianInvSE3Decoupled(tau, J_T_w_e);
        J_T_w_b = -J_T_w_e * (T_be.inverse()).Adj();

        rr_b.setIdentity();
        rr_b.topLeftCorner<3, 3>() = pp.first.rotationMatrix().transpose();
        rr_e.setIdentity();
        rr_e.topLeftCorner<3, 3>() = pp.second.rotationMatrix().transpose();

        Eigen::Matrix<double, 6, 12> J_be;
        J_be.topLeftCorner<6, 6>() = J_T_w_b * rr_b;
        J_be.topRightCorner<6, 6>() = J_T_w_e * rr_e;

        double alpha_e = config_.kinematic_constrain * m.second->lastInliers;
        const Eigen::Matrix<double, 12, 12> H12 =
            alpha_e * J_be.transpose() * J_be;
        const Eigen::Matrix<double, 12, 1> b12 =
            alpha_e * J_be.transpose() * res;
        seg_H.block<POSE_SIZE, POSE_SIZE>(0, 0) +=
            H12.topLeftCorner<POSE_SIZE, POSE_SIZE>();
        seg_H.block<POSE_SIZE, POSE_SIZE>(0, S) +=
            H12.topRightCorner<POSE_SIZE, POSE_SIZE>();
        seg_H.block<POSE_SIZE, POSE_SIZE>(S, 0) +=
            H12.bottomLeftCorner<POSE_SIZE, POSE_SIZE>();
        seg_H.block<POSE_SIZE, POSE_SIZE>(S, S) +=
            H12.bottomRightCorner<POSE_SIZE, POSE_SIZE>();
        seg_b.segment<POSE_SIZE>(0) -= b12.head<POSE_SIZE>();
        seg_b.segment<POSE_SIZE>(S) -= b12.tail<POSE_SIZE>();
      }

      // scatter the segment system into the global one
      const int oi = aom.abs_order_map.at(idx_prev).first;
      const int oj = aom.abs_order_map.at(idx_curr).first;
      abs_H.block(oi, oi, S, S) += seg_H.block(0, 0, S, S);
      abs_H.block(oi, oj, S, S) += seg_H.block(0, S, S, S);
      abs_H.block(oj, oi, S, S) += seg_H.block(S, 0, S, S);
      abs_H.block(oj, oj, S, S) += seg_H.block(S, S, S, S);
      abs_b.segment(oi, S) += seg_b.head(S);
      abs_b.segment(oj, S) += seg_b.tail(S);
    }

    // Marginalization Error Term
    // reference: Square Root Marginalization for Sliding-Window Bundle
    // Adjustment (N Demmel, D Schubert, C Sommer, D Cremers and V Usenko)
    // https://arxiv.org/abs/2109.02182
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(POSE_VEL_BIAS_SIZE);
    for (const auto& p : frame_states_) {
      if (p.second.isLinearized()) {
        delta = p.second.getDelta();
      }
    }
    const size_t M = marg_H.rows();
    abs_H.block(0, 0, M, M) += marg_H;
    abs_b.head(M) -= marg_b;
    abs_b.head(M) -= (marg_H * delta.head(M));

    if (imu_opt_active_) {
      // safety regularization: keeps vel/bias blocks nonsingular even when a
      // segment lacks its imu factor (stream gap); negligible otherwise
      for (const auto& kv : aom.abs_order_map) {
        const int idx = kv.second.first;
        abs_H.block<9, 9>(idx + 6, idx + 6).diagonal().array() += 1e-3;
      }
    }

    Eigen::VectorXd update = abs_H.ldlt().solve(abs_b);
    double max_inc = update.array().abs().maxCoeff();

    if (update.hasNaN() || !std::isfinite(max_inc)) {
      std::cerr << "[opt] NaN update! iter " << iter << " S " << S
                << " states " << aom.items << "\n  abs_H hasNaN "
                << abs_H.hasNaN() << " abs_b hasNaN " << abs_b.hasNaN()
                << " marg_H hasNaN " << marg_H.hasNaN() << "\n";
      for (auto& m : measurements) {
        std::cerr << "  seg [" << m.first.first << "," << m.first.second
                  << "] preint " << (m.second->preint ? 1 : 0)
                  << " seg_H hasNaN " << m.second->seg_H.hasNaN()
                  << " reg_H hasNaN " << m.second->delta_H.hasNaN()
                  << " inliers " << m.second->lastInliers;
        if (m.second->preint) {
          const auto& prev = frame_states_[m.first.first];
          const auto& curr = frame_states_[m.first.second];
          Eigen::Matrix<double, 9, 1> r, r_lin;
          Eigen::Matrix<double, 9, 15> J0, J1;
          m.second->preint->evaluate(prev.getState(), curr.getState(), g_w_,
                                     r);
          m.second->preint->evaluate(
              prev.isLinearized() ? prev.getStateLin() : prev.getState(),
              curr.isLinearized() ? curr.getStateLin() : curr.getState(),
              g_w_, r_lin, &J0, &J1);
          std::cerr << " r " << r.norm() << " r_lin " << r_lin.norm()
                    << " J0NaN " << J0.hasNaN() << " J1NaN " << J1.hasNaN()
                    << " W_max "
                    << m.second->preint->information().cwiseAbs().maxCoeff()
                    << " lin(" << prev.isLinearized() << ","
                    << curr.isLinearized() << ") delta0 "
                    << prev.getDelta().norm();
        }
        std::cerr << "\n";
      }
      std::exit(4);
    }

    if (max_inc < converge_thresh_) {
      break;
    }

    Eigen::Matrix<double, POSE_VEL_BIAS_SIZE, 1> inc;
    for (auto& kv : frame_states_) {
      int idx = aom.abs_order_map.at(kv.first).first;
      inc.setZero();
      inc.head(S) = update.segment(idx, S);
      kv.second.applyInc(inc);
    }
  }

  // update pseudo motion prior after each optimization
  int64_t begin_t = measurements.begin()->first.first;
  int64_t end_t = measurements.begin()->first.second;
  auto begin = frame_states_[begin_t];
  auto end = frame_states_[end_t];

  const int64_t m0_t = begin_t;
  for (auto m : measurements) {
    if (m.first.first == m0_t) continue;
    m.second->pseudoPrior = begin.getPose().inverse() * end.getPose();
    begin = frame_states_[m.first.first];
    end = frame_states_[m.first.second];
  }
}

void TrajLOdometry::Marginalize() {
  // remove pose with minimal timestamp
  if (measurements.size() >= max_frames_) {
    const size_t S = imu_opt_active_ ? POSE_VEL_BIAS_SIZE : POSE_SIZE;
    // by value, not by reference: measurements.erase(tp) below destroys the
    // very node this key lives in, and tp is still read afterwards
    const tStampPair tp = measurements.begin()->first;
    const posePair pp{frame_states_[tp.first].getPose(),
                      frame_states_[tp.second].getPose()};
    map_->Update(pp, tp);

    Eigen::VectorXd delta = frame_states_[tp.first].getDelta();

    // seg_H/seg_b hold the full two-endpoint system (registration + motion
    // factor + bias walk) at the converged linearization of Optimize()
    Eigen::MatrixXd marg_H_new = measurements.begin()->second->seg_H;
    Eigen::VectorXd marg_b_new = measurements.begin()->second->seg_b;
    marg_H_new.topLeftCorner(S, S) += marg_H;

    marg_b_new.head(S) -= marg_b;
    marg_b_new.head(S) -= (marg_H * delta.head(S));

    Eigen::MatrixXd H_mm_inv = marg_H_new.topLeftCorner(S, S).fullPivLu().solve(
        Eigen::MatrixXd::Identity(S, S));
    marg_H_new.bottomLeftCorner(S, S) *= H_mm_inv;

    marg_H = marg_H_new.bottomRightCorner(S, S);
    marg_b = marg_b_new.tail(S);
    marg_H -=
        marg_H_new.bottomLeftCorner(S, S) * marg_H_new.topRightCorner(S, S);
    marg_b -= marg_H_new.bottomLeftCorner(S, S) * marg_b_new.head(S);
    marg_H = 0.5 * (marg_H + marg_H.transpose()).eval();  // keep symmetric


    // erase
    frame_states_.erase(tp.first);
    measurements.erase(tp);

    trajectory_.emplace_back(tp.first, pp.first);
    frame_states_[tp.second].setLinTrue();
  }
}

void TrajLOdometry::PointCloudSegment(Scan::Ptr scan,
                                      Measurement::Ptr measure) {
  for (size_t i = 0; i < scan->size; i++) {
    const auto& p = scan->points[i];
    if (static_cast<int64_t>(p.ts * 1e9) < last_end_t_ns_) {
      points_cache.emplace_back(p);
    } else {
      // pub one measurement
      measure.reset(new Measurement);
      measure->tp = {last_begin_t_ns_, last_end_t_ns_};
      measure->points = points_cache;
      measure_cache.push_back(measure);

      // reset cache and time
      points_cache.clear();
      last_begin_t_ns_ = last_end_t_ns_;
      last_end_t_ns_ = last_begin_t_ns_ + window_interval_;

      if (static_cast<int64_t>(p.ts * 1e9) < last_end_t_ns_) {
        points_cache.emplace_back(p);
      }
    }
  }
}

void TrajLOdometry::RangeFilter(Measurement::Ptr measure,
                                std::vector<Eigen::Vector4d>& points) {
  points.reserve(measure->points.size());
  const auto& tp = measure->tp;
  double interv = (tp.second - tp.first) * 1e-9;
  double begin_ts = tp.first * 1e-9;
  for (const auto& p : measure->points) {
    if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
    double len = (p.x * p.x + p.y * p.y + p.z * p.z);
    if (len < min_range_2_) continue;

    double alpha = (p.ts - begin_ts) / interv;
    points.emplace_back(Eigen::Vector4d(p.x, p.y, p.z, alpha));
  }
}

void TrajLOdometry::UndistortRawPoints(std::vector<PointXYZIT>& pc_in,
                                       std::vector<PointXYZI>& pc_out,
                                       const posePair& pp) {
  Sophus::Vector6f tau =
      Sophus::se3_logd(pp.first.inverse() * pp.second).cast<float>();

  double interv = (pc_in.back().ts - pc_in.front().ts);
  double begin_ts = pc_in.front().ts;

  pc_out.reserve(pc_in.size());
  int i = 0;
  for (const auto& p : pc_in) {
    if (i % config_.point_num == 0) {
      //        float alpha = i * 1.0f / num;
      float alpha = (p.ts - begin_ts) / interv;
      Eigen::Vector3f point(p.x, p.y, p.z);
      if (point.hasNaN() || point.squaredNorm() < 4) continue;

      Sophus::SE3f T_b_i = Sophus::se3_expd(alpha * tau);
      point = T_b_i * point;
      PointXYZI po{point(0), point(1), point(2), p.intensity};
      pc_out.emplace_back(po);
    }
    i++;
  }
}

}  // namespace traj
