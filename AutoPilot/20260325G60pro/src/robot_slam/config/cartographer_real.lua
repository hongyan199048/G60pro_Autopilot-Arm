-- Cartographer 配置 for G60Pro 实车
-- 使用 Helios16 多线激光雷达，不依赖 IMU，适合实车初步建图

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",       -- 实车：不用 imu_link，直接用 base_link
  published_frame = "base_footprint", -- 与 URDF TF 树对齐：odom→base_footprint→base_link
  odom_frame = "odom",
  provide_odom_frame = true,          -- Cartographer 提供 odom 帧（map→odom→base_footprint）
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,               -- 暂不使用底盘里程计（后续可改 true）
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,               -- 使用多线点云（Helios16）
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

-- 2D 建图模式
MAP_BUILDER.use_trajectory_builder_2d = true

-- 不使用 IMU
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 每帧实时处理
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 过滤近处点：雷达在车体上，过滤掉车身自遮挡（Helios16 离地约 40cm，车身约 1.5m）
TRAJECTORY_BUILDER_2D.min_range = 1.6
TRAJECTORY_BUILDER_2D.max_range = 30.0

return options
