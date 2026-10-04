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

#include <trajlo/core/map_manager.h>

namespace traj {

std::vector<Eigen::Vector4d> MapManager::DownSampling(
    const std::vector<Eigen::Vector4d> &points, double ds_size) {
  tsl::robin_map<Voxel, Eigen::Vector4d, VoxelHash> grid;
  grid.reserve(points.size());

  for (const auto &point : points) {
    const auto voxel = Voxel((point.head<3>() / ds_size).cast<int>());
    if (grid.find(voxel) != grid.end()) continue;
    grid.insert({voxel, point});
  }
  std::vector<Eigen::Vector4d> ds_result;
  ds_result.reserve(grid.size());
  for (const auto &[voxel, point] : grid) {
    (void)voxel;
    ds_result.emplace_back(point);
  }
  return ds_result;
}
void MapManager::PreProcess(const std::vector<Eigen::Vector4d> &points,
                            const tStampPair &tp) {
  // pointcloud downsample
  map_points_database[tp] = DownSampling(points, ds_size_ * 0.5);
  reg_points_database[tp] =
      DownSampling(map_points_database[tp], ds_size_ * 1.5);
}

void MapManager::MapInit(const std::vector<Eigen::Vector4d> &points) {
  //    const auto& ds=downSampling(points, voxel_size );
  const auto &ds = points;

  std::for_each(ds.cbegin(), ds.cend(), [&](const auto &point) {
    Eigen::Vector3d p = point.head(3);
    auto voxel = Voxel((p / voxel_size_).template cast<int>());
    auto search = map.find(voxel);
    if (search != map.end()) {
      auto &voxel_block = search.value();
      voxel_block.AddPoint(p);
    } else {
      map.insert({voxel, VoxelBlock{{p}, max_points_per_voxel_}});
    }
  });
}

void MapManager::Update(const posePair &pp, const tStampPair &tp) {
  const auto &ds_points_map = map_points_database[tp];
  std::vector<Eigen::Vector3d> points_transformed(ds_points_map.size());
  Sophus::Vector6d tau = Sophus::se3_logd(pp.first.inverse() * pp.second);
  // tbb parallel for
  tbb::parallel_for(size_t(0), ds_points_map.size(), [&](size_t i) {
    Sophus::SE3d T_b_i = Sophus::se3_expd(ds_points_map[i].w() * tau);
    Sophus::SE3d T_w_i = pp.first * T_b_i;
    points_transformed[i] = T_w_i * ds_points_map[i].head<3>();
  });

  std::for_each(
      points_transformed.cbegin(), points_transformed.cend(),
      [&](const auto &point) {
        auto voxel = Voxel((point / voxel_size_).template cast<int>());
        auto search = map.find(voxel);
        if (search != map.end()) {
          auto &voxel_block = search.value();
          voxel_block.AddPoint(point);
        } else {
          map.insert({voxel, VoxelBlock{{point}, max_points_per_voxel_}});
        }
      });

  // remove from current lidar origin
  const auto max_distance2 = max_range_ * max_range_;
  for (auto it = map.begin(); it != map.end();) {
    const auto &pt = it->second.points.front();
    if ((pt - pp.first.translation()).squaredNorm() > max_distance2) {
      it = map.erase(it);
    } else {
      ++it;
    }
  }

  // remove points out of temporal window
  map_points_database.erase(tp);
  reg_points_database.erase(tp);
}

void MapManager::PointRegistrationNormal(/*const posePair& pp,*/
                                         const posePairLin &ppl,
                                         const tStampPair &tp,
                                         Eigen::Matrix<double, 12, 12> &H_icp,
                                         Eigen::Matrix<double, 12, 1> &b_icp,
                                         double &error, double &inliers) {
  const double range_thresh = 3 * reg_thresh_;
  const double th = reg_thresh_ / 3;

  auto square = [](double x) { return x * x; };
  auto Weight = [&](double residual2) {
    return square(th) / (square(th) + residual2);
  };

  // real poses (current estimate) — used for association and residual
  const posePair &pp{ppl.first.getPose(), ppl.second.getPose()};

  // Whether this segment touches a marginalized (linearized) endpoint is a
  // per-segment property, constant over every point, so the FEJ decision is
  // hoisted OUT of the per-point loop. Association and residual always use the
  // current pose; the Jacobian is linearized at the fixed point.
  //
  // Previously tangent / R_w_b_t / R_w_e_t / R_e_b were shared across the
  // parallel_reduce and overwritten in-place by an in-loop FEJ branch. That
  // both raced across threads and, within each block, contaminated the
  // current-pose residual/association of every point after the first with the
  // FEJ tangent (upstream latent bug, exposed by TBB). Splitting the two sets
  // removes the shared write entirely.
  const bool use_fej = ppl.first.isLinearized() || ppl.second.isLinearized();

  // current-pose set (association + residual)
  const Sophus::SE3d pose_b_cur = pp.first;
  const Sophus::Vector6d tangent_cur =
      Sophus::se3_logd(pp.first.inverse() * pp.second);

  // linearization-point set (Jacobian); == current when not linearized
  const Sophus::SE3d pose_b_lin = use_fej ? ppl.first.getPoseLin() : pp.first;
  const Sophus::SE3d pose_e_lin = use_fej ? ppl.second.getPoseLin() : pp.second;
  const Sophus::Vector6d tangent =
      Sophus::se3_logd(pose_b_lin.inverse() * pose_e_lin);
  const Eigen::Matrix3d R_w_b_t = pose_b_lin.rotationMatrix().transpose();
  const Eigen::Matrix3d R_w_e_t = pose_e_lin.rotationMatrix().transpose();
  const Eigen::Matrix3d R_e_b = R_w_e_t * pose_b_lin.rotationMatrix();

  const auto &ds_points_reg = reg_points_database[tp];

  // Per-point linearization (map step: neighbor search + plane fit + Jacobian).
  auto body =
      [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
        for (auto i = r.begin(); i < r.end(); ++i) {
          double alpha = ds_points_reg[i].w();
          const Sophus::SE3d T_b_i_cur = Sophus::se3_expd(alpha * tangent_cur);
          const Sophus::SE3d T_w_i_cur = pose_b_cur * T_b_i_cur;

          Eigen::Vector3d point = ds_points_reg[i].head<3>();
          Eigen::Vector3d p_in_world = T_w_i_cur * point;

          auto kx = static_cast<int>(p_in_world[0] / voxel_size_);
          auto ky = static_cast<int>(p_in_world[1] / voxel_size_);
          auto kz = static_cast<int>(p_in_world[2] / voxel_size_);
          const auto key = Voxel(kx, ky, kz);

          std::vector<Voxel> voxels;
          voxels.reserve(7);
          for (const auto &c : coord) {
            voxels.emplace_back(key + c);
          }

          std::vector<Eigen::Vector3d> neighboors;
          neighboors.reserve(7 * max_points_per_voxel_);
          std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
            auto search = map.find(voxel);
            if (search != map.end()) {
              const auto &points = search->second.points;
              if (!points.empty()) {
                for (const auto &point : points) {
                  neighboors.emplace_back(point);
                }
              }
            }
          });

          // find closet five point for normal estimation
          std::sort(neighboors.begin(), neighboors.end(),
                    [&](const Eigen::Vector3d &p1, const Eigen::Vector3d &p2) {
                      return (p1 - p_in_world).squaredNorm() <
                             (p2 - p_in_world).squaredNorm();
                    });
          if (neighboors.size() < 5) continue;
          if ((neighboors[0] - p_in_world).norm() > range_thresh) continue;

          Eigen::Matrix<double, 5, 3> A;
          Eigen::Matrix<double, 5, 1> b;
          b.setOnes();
          b *= -1.0f;
          for (int j = 0; j < 5; j++) {
            A.row(j) = neighboors[j];
          }
          Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);
          double n = normvec.norm();
          Eigen::Vector3d normal = normvec / n;
          bool flag = true;
          for (int j = 0; j < 5; j++) {
            if (fabs(normal.dot(neighboors[j]) + 1.0 / n) > planer_threshold_) {
              flag = false;
              break;
            }
          }
          if (!flag) continue;

          // point2plane use current state to compute the residual
          const Eigen::Vector3d residual = neighboors[0] - p_in_world;
          double w = Weight(residual.squaredNorm());
          Eigen::Matrix3d Information = normal * normal.transpose();

          if (normal.hasNaN() || residual.hasNaN()) {
            std::cout << "normal " << normal.transpose() << "   dis "
                      << residual.transpose() << std::endl;
            continue;
          }

          // Jacobian transform at the linearization point. tangent / R_w_b_t /
          // R_w_e_t / R_e_b were prepared once above (FEJ when the segment
          // touches a marginalized endpoint, else the current pose). Reuse the
          // current transform when not linearized to avoid a second se3_expd.
          const Sophus::SE3d T_w_i =
              use_fej ? pose_b_lin * Sophus::se3_expd(alpha * tangent)
                      : T_w_i_cur;

          Eigen::Matrix<double, 3, 6> J_T_wi;
          Eigen::Matrix<double, 6, 6> J_begin =
              Eigen::Matrix<double, 6, 6>::Zero();
          Eigen::Matrix<double, 6, 6> J_end =
              Eigen::Matrix<double, 6, 6>::Zero();
          Eigen::Matrix<double, 6, 12> J_be;

          J_T_wi.block<3, 3>(0, 0) = T_w_i.rotationMatrix();
          J_T_wi.block<3, 3>(0, 3) =
              -T_w_i.rotationMatrix() * Sophus::SO3d::hat(point);

          Eigen::Matrix3d Jr;
          Eigen::Matrix3d Jr_inv;

          Eigen::Vector3d omega = tangent.tail<3>();
          Sophus::rightJacobianSO3(alpha * omega, Jr);
          Sophus::rightJacobianInvSO3(omega, Jr_inv);

          J_end.topLeftCorner<3, 3>() =
              (Sophus::SO3d::exp((1 - alpha) * omega)).matrix() * R_w_e_t;
          J_end.bottomRightCorner<3, 3>() = Jr * Jr_inv;

          // J-begin according to the chain rule to derive the Jacobian of the begin state
          Eigen::Matrix3d R_temp = Sophus::SO3d::exp(-alpha * omega).matrix();
          J_begin.topLeftCorner<3, 3>() = (1 - alpha) * R_temp * R_w_b_t;
          J_begin.bottomRightCorner<3, 3>() =
              R_temp - alpha * Jr * Jr_inv * R_e_b;

          J_be.block<6, 6>(0, 0) = J_begin;
          J_be.block<6, 6>(0, 6) = alpha * J_end;

          Eigen::Matrix<double, 3, 12> J_r;
          J_r = J_T_wi * J_be;  // 1*3 3*6 6*12;

          J.JTJ += w * J_r.transpose() * Information * J_r;
          J.JTr += w * J_r.transpose() * Information * residual;
          J.inlier += 1;
          J.error += abs(normal.dot(residual));
        }
        return J;
      };
  auto combine = [](ResultTuple a, const ResultTuple &b) -> ResultTuple {
    return a + b;
  };

  const ResultTuple acc = tbb::parallel_reduce(
      tbb::blocked_range<size_t>{0, ds_points_reg.size()}, ResultTuple(), body,
      combine);
  const auto &[JTJ, JTr, e, num] = acc;

  H_icp = JTJ;
  b_icp = JTr;
  error = e;
  inliers = num;
}

}  // namespace traj
