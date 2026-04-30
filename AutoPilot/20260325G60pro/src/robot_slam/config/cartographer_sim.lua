-- Cartographer 配置 for G60Pro Gazebo 仿真（完全对齐实车）
-- 使用 Helios16 多线激光雷达，不依赖 IMU，与实车配置一致

include "map_builder.lua"
include "trajectory_builder.lua"

-- tracking_frame=base_footprint：Cartographer 追踪 base_footprint 位姿
-- published_frame=base_footprint：Cartographer 直接发布 map→base_footprint（跳过 odom）
-- provide_odom_frame=false：不发布 odom 中间层，避免与 Gazebo 的 odom→base_footprint 冲突
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",    -- 追踪 base_footprint（与 Gazebo 一致）
  published_frame = "base_footprint",   -- 直接发布 map→base_footprint
  odom_frame = "odom",
  provide_odom_frame = false,           -- 不发布 odom，避免 TF 冲突
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,                  -- 启用 odom 数据（Gazebo 提供准确 odom）
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                 -- 使用多线点云（Helios16）
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

-- 不使用 IMU（仿真 IMU 数据质量差）
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 每帧实时处理
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 距离过滤：Gazebo 仿真无自遮挡，降低 min_range
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 30.0

-- Z 方向高度过滤（相对 tracking_frame = base_footprint，地面 Z=0）
-- 有效障碍物高度范围（相对地面）：0.1m ~ 2.0m
TRAJECTORY_BUILDER_2D.min_z = 0.1
TRAJECTORY_BUILDER_2D.max_z = 2.0

-- 运动滤波：Gazebo odom 准确，按距离/角度触发
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.017  -- ~1°

-- 在线相关扫描匹配：Gazebo odom 准确，可以降低搜索窗口
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(10.)  -- 降低到 10°
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 扫描匹配权重（与实车一致，改善墙壁双线和旋转错位）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400

return options
