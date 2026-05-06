-- Cartographer 纯定位模式配置 for G60Pro 实车
-- 完全复制 cartographer_real.lua（SLAM 配置），只改加载 pbstream 必需的项
-- 用 SLAM 验证过的参数，排除参数差异的影响
-- TF: map→base_footprint 直接发布，odom frame 不出现在 TF 树中

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,           -- odom frame 不出现在 TF 树中，直接 map→base_footprint
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,            -- 与 SLAM 一致
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,         -- 纯定位地图固定，不需要高频发布子图
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 1.6
TRAJECTORY_BUILDER_2D.max_range = 30.0
-- Z 方向高度过滤（相对 tracking_frame = base_footprint，地面 Z=0）
TRAJECTORY_BUILDER_2D.min_z = 0.1
TRAJECTORY_BUILDER_2D.max_z = 2.0  -- rs16_link 水平光束 z≈1.54m（base_footprint系），需要 >1.54

-- 以下全部与 SLAM 配置完全一致
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.017

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400

return options
