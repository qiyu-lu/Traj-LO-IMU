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

#ifndef TRAJLO_ODOMETRY_H
#define TRAJLO_ODOMETRY_H

#include <atomic>
#include <memory>
#include <thread>

#include <tbb/concurrent_queue.h>
#include <Eigen/Eigen>

#include <trajlo/utils/common_type.h>
#include <trajlo/utils/config.h>
#include <trajlo/utils/pose_type.h>
#include <trajlo/utils/eigen_utils.hpp>

#include <trajlo/core/imu_preintegration.h>
#include <trajlo/core/map_manager.h>

namespace traj {
class TrajLOdometry {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<TrajLOdometry>;

  TrajLOdometry(const TrajConfig& config);
  ~TrajLOdometry() {
    processing_thread_->join();
    std::cout << "processing is finished !!!" << std::endl;
  }

  void Start();
  void Optimize();
  void Marginalize();
  void PointCloudSegment(Scan::Ptr scan, Measurement::Ptr measure);
  void RangeFilter(Measurement::Ptr measure, std::vector<Eigen::Vector4d>& points);
  void UndistortRawPoints(std::vector<PointXYZIT>& pc_in,
                          std::vector<PointXYZI>& pc_out, const posePair& pp) ;

  tbb::concurrent_bounded_queue<Scan::Ptr> laser_data_queue;
  tbb::concurrent_bounded_queue<ImuData::Ptr> imu_data_queue;
  std::atomic<bool> isFinish;  // for feed data thread join

  tbb::concurrent_bounded_queue<ScanVisData::Ptr>* vis_data_queue = nullptr;

 private:
  MapManager::Ptr map_;
  bool isMove_ = false;

  Sophus::SE3d T_wc_curr;
  Sophus::SE3d T_prior;  // motion prior
  Eigen::aligned_map<tStampPair, Measurement::Ptr> measurements;
  std::deque<Measurement::Ptr> measure_cache;
  std::vector<PointXYZIT> points_cache;

  double min_range_2_=1.0;
  double converge_thresh_=0.01;

  // state database (pose+vel+bias per segment endpoint; vel/bias only get
  // columns in the optimization when imu_opt_active_ is set)
  bool imu_stream_en_ = false;   // data path: consume the imu queue
  bool imu_opt_active_ = false;  // optimization: 15-dim states + imu factors
                                 // (flipped once static init completes)
  Eigen::aligned_map<int64_t, PoseVelBiasStateWithLin<double>> frame_states_;

  // imu stream buffering (fed by the data loader in bag time order)
  void FetchImuUntil(int64_t t_ns);
  ImuData InterpImuAt(int64_t t_ns) const;
  ImuPreintegration::Ptr BuildSegmentPreint(const tStampPair& tp);
  void TryFinishImuInit(const tStampPair& tp);
  std::deque<ImuData::Ptr> imu_buffer_;
  bool imu_stream_ended_ = false;
  size_t imu_total_count_ = 0;

  // imu-to-lidar sample transform (samples are stored in the lidar frame)
  Sophus::SO3d R_l_i_;      // rotation imu->lidar
  Eigen::Vector3d t_i_l_;   // lidar origin in imu frame (lever arm)
  int64_t imu_time_offset_ns_ = 0;

  // static initialization & gravity
  StaticInitializer static_init_;
  int64_t init_fed_until_ = 0;
  Eigen::Vector3d g_w_ = Eigen::Vector3d(0, 0, -9.81);
  using tumPose = std::pair<int64_t, Sophus::SE3d>;
  std::vector<tumPose> trajectory_;
  //  std::vector<Sophus::SE3d> trajectory;

  // slide window parameters
  int64_t window_interval_ = 4e7;
  int64_t init_interval_ = 3e8;
  size_t max_frames_ = 4;
  int64_t last_begin_t_ns_;
  int64_t last_end_t_ns_;


  // marginalization
  AbsOrderMap marg_order;
  Eigen::MatrixXd marg_H;
  Eigen::VectorXd marg_b;

  TrajConfig config_;

  // backend thread
//  bool initialized_;
  std::shared_ptr<std::thread> processing_thread_;
};

}  // namespace traj

#endif  // TRAJLO_ODOMETRY_H
