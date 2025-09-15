Simulation-related params
---
|                    |                                          **Definition/Purpose**                                          |                                              **Value**                                             |
|--------------------|:--------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------:|
| **lid_topic**      |                                        input topic for LiDAR data                                        |                                      e.g. "/scan_points_fixed"                                     |
| **imu_topic**      |                                         input topic for IMU data                                         |                                       e.g. "/livox/imu_fixed"                                      |
| **lidar_type**     |  # 1 for Livox serials LiDAR, 2 for Velodyne LiDAR, 3 for ouster LiDAR, 4 for any other pointcloud input |                                   Set to **4** in simulation mode                                  |
| **fov_degree**     | Sets the vertical field of view for the LiDAR feature selection and local map segmentation in FAST-LIO2. |                                  For Livox Mid 360, set to **60**                                  |
| **det_range**      |                         Sets the maximum detection (mapping) range for FAST-LIO2.                        | For Livox Mid 360, set to something between  **50** and **60** (80–90% of actual LiDAR max range). |
| **scan_line**      |                Specifies the number of vertical channels (i.e., scan lines) in the LiDAR.                |  Set to **1** for Livox Mid 360 Set to **16** for Velodyne VLP-16 Set to **64** for Ouster OS1-64  |
| **blind**          |            Minimum distance threshold (in meters) under which noisy near returns are ignored.            |                                    Set to **0.15** in simulation                                   |
| **scan_rate**      |                       Specifies the LiDAR scan frequency in Hz (scans per second).                       |                For Livox Mid 360, set to **10 Hz**. In simulation match plugin rate.               |
| **timestamp_unit** |         Specifies the unit of the timestamp inside each LiDAR point (not the ROS message header).        |                          Set to **0** (Seconds) For ROS/Gazebo simulations                         |)

Sensor calibration and synchronization
---
### Calibration
Please refer to the manufacturer's data sheet for information on the exact extrinsic transformation values between the sensors used in the SLAM algorithm.

For **FAST-LIO2** (using **Livox Mid 360** LiDAR with integrated IMU sensor) use following values: 
* **extrinsic_T**: [ -0.011, -0.02329, 0.04412 ]
* **extrinsic_R**: [ 1., 0., 0.,
                  0., 1., 0.,
                  0., 0., 1.]

### Synchronization

* **time_sync_en**: enables internal IMU-LiDAR time offset correction.
* **time_offset_lidar_to_imu**: used for compensation if the sensor delay is known precisely.
* **extrinsic_est_en**: enables online estimation of the rigid-body transform (rotation + translation) between LiDAR and IMU.

   |                              | **Simulation** |        **Real-world**        |
   |------------------------------|:--------------:|:----------------------------:|
   | **time_sync_en**             |     false*     |             true             |
   | **time_offset_lidar_to_imu** |      0.0**     |     0.0 or measured value    |
   | **extrinsic_est_en**         |    false***    | true (if extrinsics unknown) |

*: Simulated data is usually already synchronized via /clock.

**: Simulated sensors are timestamped on publish, so offset is near zero.

***: Exact static transforms are defined, so no need to estimate.


SLAM-related params
---

|                            |                                                                    **Definition/Purpose**                                                                    |                                                          **Value/Impact**                                                          |
|----------------------------|:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------------------------:|
| **feature_extract_enable** |  Enables or disables feature-based SLAM.                                                                                                                     | **true**: enable feature-based SLAM ------------------------------------------------- **false**: pointcloud is passed through, but no SLAM |
| **point_filter_num**       |  Filters out near-LiDAR noise or reflective surfaces (e.g., drone body, rotors).                                                                             | **1-2**: recommended value  -------------------------------------------------  **0** : no filtering (keep all points)                      |
| **max_iteration**          | Number of iterative EKF update steps per LiDAR frame (more steps → better state convergence at CPU cost).                                                    | **3** is a balanced default                                                                                                            |
| **filter_size_surf**       | Downsamples voxels for surface feature points per scan before SLAM.  Smaller = more features → higher accuracy, higher CPU and vice-versa.                   | Set to **0.25** as initial balanced value                                                                                              |
| **filter_size_map**        | Downsamples the global map and affects stored/saved pointcloud size, not SLAM internals directly                                                             | Set to **0.25** as initial value                                                                                                       |
| **cube_side_length**       | Size of the working map region around the drone. Affects the memory usage heavily. E.g. cube_side_length = 1000.0 means a region of 1000 m × 1000 m × 1000 m | Typical indoor value: **100.0** or **200**.0                                                                                               |
| **dense_publish_en**       | Controls the published/saved map.                                                                                                                            | Set to **false** to reduce .pcd size substantially                                                                                     |

Map-related params
---

### Enable and tune map recording
1. Set "**pcd_save_en**" to true
2. Set "**interval**" to a something between 5-10. If you want to save all frames set to -1 (Memory exhausting).
3. Set "**dense_publish_en**" to false to keep key features only and get rid of unnecessary LiDAR points.

### Save recorded map
1. Modify parameter "**map_file_path**" to desired file name. Default name is "**"./test.pcd"**".
   - (HINT) Recorded map has **.pcd** format
2. Call "**/map_save**" ros service in separate terminal to save recorded map:
```bash
   ros2 service call /map_save std_srvs/srv/Trigger
```
   - (HINT) File will be saved in the directory from which you call the /map_save ros service.