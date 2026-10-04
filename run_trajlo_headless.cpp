/**
 * Headless entry-point for Traj-LO (no ImGui/GLFW window).
 *
 * The stock run_trajlo.cpp only starts odometry after a human toggles the
 * "Odometry" window and clicks "Start" in the GUI, so it cannot run unattended
 * in a batch harness. This variant wires the data loader straight into the
 * odometry queue, starts both, and blocks until the odometry worker has
 * consumed the whole bag (it saves the TUM trajectory to `pose_file_path` on
 * data-end and sets isFinish=true). The process then exits on its own, which
 * is exactly the "bag finished" signal bench_all/run_all.py waits for.
 *
 * Usage:  trajlo_headless <config_path>
 */

#include <trajlo/core/odometry.h>
#include <trajlo/io/data_loader.h>
#include <trajlo/utils/config.h>

#include <chrono>
#include <iostream>
#include <thread>

int main(int argc, char *argv[]) {
  std::cout << "Traj-LO (headless) starting\n";

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config_path>" << std::endl;
    return 1;
  }

  traj::TrajConfig config;
  config.load(argv[1]);

  traj::TrajLOdometry::Ptr odometry(new traj::TrajLOdometry(config));
  traj::DataLoader::Ptr dataLoader =
      traj::DatasetFactory::GetDatasetIo(config.type);
  if (!dataLoader) {
    std::cerr << "Unknown data_type: " << config.type << std::endl;
    return 2;
  }

  // Wire the loader's output queue to the odometry input queue BEFORE any data
  // flows (this is what the GUI does when "Start" is pressed).
  dataLoader->laser_queue = &odometry->laser_data_queue;
  if (config.imu_en && !config.imu_topic.empty()) {
    dataLoader->imu_queue = &odometry->imu_data_queue;
    dataLoader->imu_topic = config.imu_topic;
  }

  odometry->Start();    // spawn the processing thread (blocks on the queue)
  dataLoader->start();  // is_pub_ = true -> loader may push scans

  // The loader reads the whole bag, pushes each scan, then pushes a nullptr
  // sentinel; the odometry worker breaks on the sentinel, saves the pose file
  // and sets isFinish = true.
  std::thread t_io(&traj::DataLoader::publish, dataLoader, config.dataset_path,
                   config.topic);

  while (!odometry->isFinish) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  t_io.join();
  std::cout << "Traj-LO (headless) finished\n";
  return 0;
}
