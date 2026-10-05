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

#ifndef TRAJLO_CONFIG_H
#define TRAJLO_CONFIG_H

#include <string>
#include <vector>

#include <sophus/se3.hpp>

namespace traj {
struct TrajConfig {
  TrajConfig() = default;
  void load(const std::string& filename);

  // dataset
  std::string type;
  std::string topic;
  std::string dataset_path;

  bool save_pose;
  std::string pose_file_path;

  // calibration:
  double time_offset;
  Sophus::SE3d T_body_lidar;
  Sophus::SE3d T_body_gt;

  // imu (optional section; imu_en=false when absent)
  // noise defaults are deliberately inflated far beyond datasheet values:
  // on legged robots foot-impact vibration is unmodeled measurement noise
  // (validated on legkilo/corridor, M4 gate)
  bool imu_en = false;
  std::string imu_topic;
  double sigma_ng = 5e-2;     // gyro noise density [rad/s/sqrt(Hz)]
  double sigma_na = 5e-1;     // accel noise density [m/s^2/sqrt(Hz)]
  double sigma_bg_rw = 1e-4;  // gyro bias random walk
  double sigma_ba_rw = 1e-3;  // accel bias random walk
  double imu_weight = 0.1;    // global lidar-vs-imu balance knob
  double imu_init_time = 0.8; // static initialization span [s]

  // trajectory
  double init_interval;
  double seg_interval;
  int seg_num;
  float kinematic_constrain;
  double init_pose_weight;
  double converge_thresh;
  int max_iterations;

  // mapping
  float ds_size;
  float voxel_size;
  int max_voxel_num;
  float planer_thresh;
  float max_range;
  float min_range;
  // map structure: "voxel" (Traj-LO's voxel hash) or "kdtree" (likd-tree)
  std::string map_type = "voxel";
  float kd_min_dist = 0.1;     // skip a new map point closer than this to the map
  float kd_max_nn_dist = 1.0;  // drop k-NN neighbors farther than this

  // vis
  int frame_num;
  int point_num;
  Sophus::SE3d T_vis_lidar;
};
}  // namespace traj

#endif  // TRAJLO_CONFIG_H
