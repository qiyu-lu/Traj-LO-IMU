<div align="center">
    <h1>Traj-LO-IMU</h1>
    <i>Traj-LO with IMU preintegration factors</i>
    <br>
    <br>
</div>

> [!IMPORTANT]
> This is an **unofficial** extension of [Traj-LO](https://github.com/kevin2431/Traj-LO)
> by Xin Zheng and Jianke Zhu. It is **not** the implementation or a reproduction of
> [Traj-LIO (Zheng & Zhu, 2024)](https://arxiv.org/abs/2402.09189), which estimates a
> sparse Gaussian-process trajectory for multi-LiDAR multi-IMU systems.
> This repository keeps Traj-LO's piecewise-linear continuous-time trajectory and only
> adds IMU preintegration to its sliding-window optimization. There is no new method
> here; all credit for the core LiDAR odometry goes to the original authors.

<div align="center">
    <img src="doc/image/trajectory.png" width="50%" height="auto" alt="Trajectory Image">
    <img src="doc/image/pipeline.png" width="40%" height="auto" alt="Pipeline Image">
</div>

## What is Traj-LO

[Traj-LO](https://ieeexplore.ieee.org/document/10387726) explores the limits of state estimation using only LiDAR.
The spatial-temporal movement of the LiDAR is parameterized by a continuous-time trajectory made of multiple piecewise linear functions.
By coupling geometric information from streaming LiDAR points with kinematic constraints from trajectory smoothness, it matches state-of-the-art LIO methods in many scenarios
([video](https://youtu.be/hbtKzElYKkQ?si=ZlqvtUVhhJbAju0S)).

## What this fork adds

Being LiDAR-only, Traj-LO can drift or diverge where geometry is degenerate for long periods (narrow corridors, stairwells, open fields).
This fork adds an optional IMU path to improve robustness in those cases:

- **State**: each trajectory knot carries pose, velocity and gyro/accel biases (pose-only when IMU is disabled, so the LiDAR-only path is unchanged).
- **IMU data path**: IMU messages are read from the same rosbag (`imu.topic`) and queued alongside the point clouds.
- **Preintegration**: IMU measurements between adjacent knots are preintegrated, with analytic Jacobians and bias-correction terms (`include/trajlo/core/imu_preintegration.h`, checked by `test/test_imu_jacobians.cpp`).
- **Static initialization**: gravity direction, gyro bias and the measured gravity magnitude are estimated from the first `imu.init_time` seconds, which must be static.
- **Sliding-window fusion**: preintegration and bias random-walk factors are added to the Gauss-Newton window and handled by marginalization, together with Traj-LO's LiDAR point-to-plane and kinematic terms.
- **Headless runner**: `trajlo_headless <config>` runs without the GUI, useful for benchmarking.
- **Bug fixes to upstream code**:
  - a data race in `MapManager::PointRegistrationNormal`, where variables shared across the TBB `parallel_reduce` were overwritten by the FEJ branch, which also corrupted residuals and associations
  - a use-after-free in `TrajLOdometry::Marginalize`

In our tests on legged-robot datasets (Leg-KILO, Quadruped-SLAM, DiTer++) the IMU mode mainly improves **robustness**: sequences where LiDAR-only Traj-LO diverged (e.g. stairwells) now converge.
Accuracy on sequences where LiDAR-only already works is similar. Your mileage may vary, and the parameters will need tuning for your platform.

### Enabling the IMU

Add an `imu` section to the config (see `data/config_lio_*.yaml`). Without it, the code runs as plain LiDAR-only Traj-LO.

```yaml
imu:
  enabled: true
  topic: "/imu_raw"
  sigma_ng: 5e-2      # gyro noise density [rad/s/sqrt(Hz)]
  sigma_na: 5e-1      # accel noise density [m/s^2/sqrt(Hz)]
  sigma_bg_rw: 1e-4   # gyro bias random walk
  sigma_ba_rw: 1e-3   # accel bias random walk
  imu_weight: 0.1     # global LiDAR-vs-IMU balance
  init_time: 0.8      # static initialization span [s]
```

`calibration.T_body_lidar` must be the LiDAR-to-IMU extrinsic (`p_imu = R * p_lidar + t`).

Notes:
- The provided noise densities are deliberately inflated, because foot impacts on legged robots act as unmodeled vibration noise. On wheeled or handheld platforms, values closer to the datasheet may work better.
- The gravity magnitude is taken from the measured mean accelerometer norm during initialization rather than fixed at 9.81, since some consumer IMUs have a ~1% scale error.

| Config file | Dataset |
|---|---|
| `config_lio_legkilo.yaml` | [Leg-KILO](https://github.com/ouguangjun/Leg-KILO) (Unitree, VLP-16) |
| `config_lio_quadruped.yaml` | Quadruped-SLAM (Unitree Go2, Velodyne) |
| `config_lio_diter.yaml`, `config_lio_diter_park.yaml` | DiTer++ (Ouster + built-in IMU) |
| `config_lio_ouster.yaml`, `config_lio_velodyne.yaml` | Hilti 2021 / SubT-MRS with IMU |

## How to use
Traj-LO is a ROS-independent project and is suitable for cross-platform applications. For convenience, we provide a ROSbag data loader that can read public datasets and your own recorded data.

### Supported Dataset
Currently, the released code only supports one LiDAR configuration. We will update it as soon as possible to provide multi-LiDAR support. The provided ROSbag data loader supports different types of LiDAR, including Livox, Ouster, Hesai, Robosense, and Velodyne. We have tested Traj-LO with the following datasets.

| Dataset                                                                                                                                                                                                                 | LiDAR                               | Message Type                                       | Configuration file                       |
|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------|----------------------------------------------------|------------------------------------------|
| [NTU VIRAL](https://ntu-aris.github.io/ntu_viral_dataset/)                                                                                                                                                              | Two Ouster OS1-16                   | sensor_msgs/PointCloud2                            | config_ntu.yaml                          |
| [Hilti 2021](https://www.hilti-challenge.com/dataset-2021.html)                                                                                                                                                         | Ouster OS0-64 <br/> Livox mid-70    | sensor_msgs/PointCloud2 <br/>livox_ros_driver/CustomMsg | config_ouster.yaml<br/>config_livox.yaml |
| [R3LIVE](https://github.com/ziv-lin/r3live_dataset)                                                                                                                                                                     | Livox Avia                          | livox_ros_driver/CustomMsg                            | config_livox.yaml                        |
| [Point-LIO](https://connecthkuhk-my.sharepoint.com/personal/hdj65822_connect_hku_hk/_layouts/15/onedrive.aspx?id=%2Fpersonal%2Fhdj65822%5Fconnect%5Fhku%5Fhk%2FDocuments%2FDataset%20for%20Point%2DLIO%20examples&ga=1) | Livox Avia                          | livox_ros_driver/CustomMsg                            | config_pointlio.yaml                     |
| [New College](https://ori-drs.github.io/newer-college-dataset/)                                                                                                                                                         | Ouster OS-1 64 <br/>Ouster OS-0 128 | sensor_msgs/PointCloud2                            | config_ouster.yaml                       |
| [Hilti 2022 &2023](https://www.hilti-challenge.com/dataset-2022.html)                                                                                                                                                    | Hesai PandarXT-32 | sensor_msgs/PointCloud2                            | config_hesai.yaml                        |
| [SubT-MRS](https://superodometry.com/iccv23_challenge_LiI)                                                                                                                                                    | Velodyne VLP16 | sensor_msgs/PointCloud2                            | config_velodyne.yaml                     |

The corresponding configuration files are located in the "data" directory. For optimal performance, you will need to fine-tune the parameters.

Since Traj-LO is a LiDAR-only method, it may fail in narrow spaces where there are few valid points for a long time.

### Dependency
In addition to the ROSbag data loader, Traj-LO also provides a simple custom GUI for visualization and uses Eigen-based Gauss-Newton for pose optimization. Here are the major libraries we will use.
- Optimization: [Eigen](https://gitlab.com/libeigen/eigen.git), [Sophus](https://github.com/strasdat/Sophus.git)
- GUI: [ImGui](https://github.com/ocornut/imgui), OpenGL, [GLM](https://github.com/g-truc/glm.git)
- DataLoader: oneTBB, Boost

Although major dependencies are included in the third-party folder, you may still need to run the script `install_deps.sh` to install libraries like Boost, etc.
### Build
You can install the Traj-LO project by following these steps:
```bash
git clone https://github.com/qiyu-lu/Traj-LO-IMU.git
cd Traj-LO-IMU
./scripts/install_deps.sh # make sure we have all the dependency
mkdir build && cd build
# oneTBB's CMake may fail under a non-English locale; if so, run first:
# export LANG=C LC_ALL=C
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
```



### Run
After modifying the config file for your environment, you can run Traj-LO. Here is an example to test it with a Livox LiDAR.
```
./trajlo ../data/config_livox.yaml          # LiDAR-only, with GUI
./trajlo ../data/config_lio_legkilo.yaml    # with IMU, with GUI
./trajlo_headless ../data/config_lio_legkilo.yaml  # without GUI
```

### Some Tips
- Traj-LO is a continuous-time method, so each point in your rosbag should have a corresponding timestamp.
- When the motion profile is aggressive, you can decrease `seg_interval` or increase `kinematic_constraint`.

## Cross-platform Support
### Linux
Ubuntu 20.04, 22.04
### Windows
You can use [WSL2](https://learn.microsoft.com/zh-cn/windows/wsl/about) to install the Ubuntu subsystem and then follow the above instructions to test Traj-LO. To enable OpenGL accelerated rendering in WSLg, you may need to [select Nvidia GPU](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg).
### MacOS
Make sure you have [Homebrew](https://brew.sh/) to run the srcipt `install_deps.sh`to install dependencies. We have tested Traj-LO on M2 Mac Mini (macOS 14.4.1).
### ROS
Still working on it!

## Citation

This repository is only an engineering extension. If you use it in academic work, please cite the original Traj-LO paper:

```bibtex
@ARTICLE{zheng2024traj,
    author={Zheng, Xin and Zhu, Jianke},
    journal={IEEE Robotics and Automation Letters},
    title={Traj-LO: In Defense of LiDAR-Only
    Odometry Using an Effective Continuous-Time
    Trajectory},
    year={2024},
    volume={9},
    number={2},
    pages={1961-1968},
    doi={10.1109/LRA.2024.3352360}
}
```

For a principled multi-LiDAR multi-IMU continuous-time estimator from the same authors, see [Traj-LIO](https://arxiv.org/abs/2402.09189).

## License

MIT, same as the upstream Traj-LO. The original copyright notice is kept in [LICENSE](LICENSE). Third-party libraries in `thirdparty/` keep their own licenses.

## Acknowledgement
This work is built entirely on [Traj-LO](https://github.com/kevin2431/Traj-LO). Upstream Traj-LO thanks these pioneering works: these pioneering works [Basalt](https://cvg.cit.tum.de/research/vslam/basalt) (Batch Optimization), [CT-ICP](https://github.com/jedeschaud/ct_icp) (Continuous-time Idea), and [KISS-ICP](https://github.com/PRBonn/kiss-icp) (VoxelMap Management).

