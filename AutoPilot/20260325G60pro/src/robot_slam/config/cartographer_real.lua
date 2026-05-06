-- Cartographer 配置 for G60Pro 实车
-- 使用 Helios16 多线激光雷达，不依赖 IMU，适合实车初步建图

include "map_builder.lua"
include "trajectory_builder.lua"

-- tracking_frame=base_footprint：Cartographer 追踪地面投影点，map Z=0 对齐地面
-- published_frame=base_footprint：Cartographer 直接发布 map→base_footprint
-- provide_odom_frame=false：实车无里程计，不插入 odom 帧，避免与 robot_state_publisher 冲突
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",    -- 追踪地面投影点，与 sim 一致
  published_frame = "base_footprint",   -- 直接发布 map→base_footprint
  odom_frame = "odom",
  provide_odom_frame = false,           -- 无真实里程计，不插入 odom 帧
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,                -- 暂不使用底盘里程计（后续可改 true）
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                -- 使用多线点云（Helios16）
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

-- 距离过滤：过滤车身自遮挡（车身对角约 1.5m）和远端噪声
TRAJECTORY_BUILDER_2D.min_range = 1.6
TRAJECTORY_BUILDER_2D.max_range = 30.0

-- Z 方向高度过滤（相对 tracking_frame = base_footprint，地面 Z=0）
-- 有效障碍物高度范围：0.1m ~ 2.0m（与仿真一致）
-- rs16_link 水平光束在 base_footprint 系中 z≈1.54m，max_z=2.0 可保留
TRAJECTORY_BUILDER_2D.min_z = 0.1
TRAJECTORY_BUILDER_2D.max_z = 2.0

-- 运动滤波：有 odom 先验后按距离/角度触发，降低多余帧插入
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.017  -- ~1°

-- 在线相关扫描匹配（旋转鲁棒性关键）：
-- Ceres 是局部优化器，旋转时 odom 无效（use_odometry=false）导致初始位姿偏差，
-- 容易陷入错误极小值。RTCSM 先在搜索窗口内暴力全局搜索最优初始位姿，再交 Ceres 精化。
-- 实车无 odom 无 IMU，位姿外推无速度先验，搜索窗口需覆盖帧间实际运动量
-- 20° 与仿真一致；若 CPU 压力大可降至 15°
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 扫描匹配权重（与仿真一致，改善墙壁双线和旋转错位）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400

return options
