-- Cartographer 纯定位模式配置 for G60Pro 实车
-- 加载 .pbstream 地图文件，在已保存地图内做 scan matching 定位
-- 不再构建新地图

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",         -- 追踪车体中心
  published_frame = "base_footprint",   -- 发布 map→base_footprint
  odom_frame = "odom",
  provide_odom_frame = false,           -- 不插入 odom 帧
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,                 -- Helios16 多线点云
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- 纯定位模式：加载已保存地图，不构建新地图
-- 注意：pure_localization 由 -load_state_filename 参数自动启用，无需在 Lua 中设置
MAP_BUILDER.use_trajectory_builder_2d = true

-- 不使用 IMU
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 每帧实时处理
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 距离过滤（与 cartographer_real.lua 保持一致）
TRAJECTORY_BUILDER_2D.min_range = 1.6
TRAJECTORY_BUILDER_2D.max_range = 30.0

-- 高度过滤（相对 base_link）
TRAJECTORY_BUILDER_2D.min_z = -0.6
TRAJECTORY_BUILDER_2D.max_z = 1.4

-- 运动滤波（纯定位模式：提高更新频率，适应快速转弯）
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.1   -- 0.1秒强制发布（10Hz）
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01  -- 1cm发布
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.5)  -- 0.5°发布（快速转弯时高频更新）

-- 在线相关扫描匹配（纯定位模式：适度搜索窗口，平衡精度和稳定性）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3  -- 30cm搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)  -- 20°搜索窗口
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.0  -- 默认惩罚权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1.0     -- 默认惩罚权重

-- 扫描匹配权重（纯定位模式：适度提高权重，避免过度强制对齐）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20   -- 适度提高（原10，之前50太高）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 200     -- 适度提高（原100，之前500太高）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 800        -- 适度提高（原400，之前2000太高）

return options