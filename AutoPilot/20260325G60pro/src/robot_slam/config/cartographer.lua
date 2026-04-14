-- Cartographer 配置 for G60Pro (多线激光雷达 Helios16 - 2D模式)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,          -- 禁用单线激光扫描
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,         -- 启用多线点云（Helios16）
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.05,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 启用 2D 模式
MAP_BUILDER.use_trajectory_builder_2d = true

-- 启用 IMU（tracking_frame=imu_link 满足共位约束，publish_frame_projected_to_2d=true 保证地图在地面）
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 每帧实时处理，避免积攒多帧后产生跳变
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 过滤近处点云：雷达在车头0.7m，车身对角约1.5m，设1.6m过滤自身车体
TRAJECTORY_BUILDER_2D.min_range = 1.6

return options
