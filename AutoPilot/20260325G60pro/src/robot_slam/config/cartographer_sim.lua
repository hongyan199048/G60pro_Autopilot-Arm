-- Cartographer 配置 for G60Pro Gazebo 仿真
--
-- TF 分工（无冲突）：
--   Cartographer  → map → odom            （SLAM 全局修正）
--   gazebo planar → odom → base_footprint  （Gazebo 地面真值）
--   robot_state   → base_footprint → base_link → rs16_link 等（URDF 静态链）
--
-- tracking_frame = base_footprint：
--   base_footprint 在 odom 中 Z=0，不依赖 imu_link 的 Z 偏移，
--   且不受 imu 噪声影响，避免 map→odom 产生 Z 漂移。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",   -- 仿真：base_footprint Z=0，稳定，无 IMU 噪声干扰
  published_frame = "odom",            -- Cartographer 发布 map→odom
  odom_frame = "odom",
  provide_odom_frame = false,          -- false：只发布 map→odom，避免与 Gazebo 发布 odom→base_footprint 冲突
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,                 -- 使用 Gazebo planar_move 发布的真值 odom，提供姿态外推先验，避免扫描匹配失败
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
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

-- 2D 建图
MAP_BUILDER.use_trajectory_builder_2d = true

-- tracking_frame=base_footprint，不需要 IMU，避免 Z 轴噪声
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 每帧实时处理
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 距离过滤：最小 0.5m（仿真无自遮挡问题）
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 30.0

-- Z 方向高度过滤（相对 tracking_frame = base_footprint，地面 Z=0）
-- min_z=0.1：过滤地面回波；max_z=2.5：保留有效障碍物高度
TRAJECTORY_BUILDER_2D.min_z = 0.1
TRAJECTORY_BUILDER_2D.max_z = 2.5

-- 运动滤波：有 odom 先验后可用距离/角度触发，降低多余帧插入
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.017  -- ~1°

-- 在线相关扫描匹配（旋转鲁棒性关键）：
-- Ceres 是局部优化器，初始位姿偏差时（旋转场景下 odom 有延迟）容易陷入错误极小值。
-- RTCSM 先在搜索窗口内暴力全局搜索最优初始位姿，再交 Ceres 精化，显著改善旋转时的错位问题。
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 扫描匹配权重
-- occupied_space_weight：当前帧与已有地图的对齐权重，提高可减少沿墙滑动导致的双线
-- translation/rotation_weight：位姿先验约束，与 occupied_space_weight 同比例提高
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400

return options
