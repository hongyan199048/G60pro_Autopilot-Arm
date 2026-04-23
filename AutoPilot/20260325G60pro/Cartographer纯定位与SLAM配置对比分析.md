# Cartographer 纯定位 vs SLAM 配置分析

## 背景

G60Pro 实车使用 Cartographer 做 2D SLAM 建图，建图效果良好：点云实时更新、定位准确、CPU 占用正常。但使用纯定位模式（加载 .pbstream 地图）时，出现以下问题：

- 点云更新慢（0.5~1s 才刷新一次）
- 小车一动定位就丢失
- CPU 占用率超过 30%

经排查，原因为纯定位配置的参数与 SLAM 配置存在多处不合理差异，导致扫描匹配效率低下且质量差。

## 参数对比

| 参数 | SLAM 配置 | 原始纯定位配置 | 差异 |
|------|-----------|---------------|------|
| `use_pose_extrapolator` | `true` | **`false`** | 关闭了位姿预测 |
| `motion_filter.max_time_seconds` | `5.0` | **`0.1`** | 50 倍激进 |
| `motion_filter.max_distance_meters` | `0.2` | **`0.01`** | 20 倍激进 |
| `motion_filter.max_angle_radians` | `0.017` (~1°) | `0.017` (~1°) | 相同 |
| `linear_search_window` | `0.1` | **`0.3`** | 3 倍 |
| `angular_search_window` | `20°` | **`30°`** | 1.5 倍 |
| `occupied_space_weight` | `10` | **`20`** | 2 倍 |
| `translation_weight` | `100` | **`200`** | 2 倍 |
| `rotation_weight` | `400` | **`800`** | 2 倍 |

其余参数（`tracking_frame`、`published_frame`、`use_odometry`、`min_range`、`max_range`、`min_z`、`max_z`、`num_accumulated_range_data` 等）两者一致。

## 问题根因分析

原始纯定位配置的多个参数形成了恶性循环：

### 1. motion_filter 过于激进 → 匹配频率爆炸

运动滤波器阈值极低（`max_time=0.1s`、`max_distance=0.01m`），导致几乎每帧点云都触发完整扫描匹配。Cartographer 以远超正常频率的速度进行匹配计算，大量 CPU 时间消耗在不必要的匹配上。

### 2. 关闭 pose_extrapolator → 每次匹配从零开始

`use_pose_extrapolator = false` 意味着 Cartographer 没有位姿预测能力。每帧扫描匹配都需要从上一帧位置开始搜索，无法利用运动连续性给出好的初始猜测。这迫使 RTCSM 做更大范围的暴力搜索。

### 3. 搜索窗口过大 → 单次匹配计算量大

`linear_search_window = 0.3m`（SLAM 的 3 倍）和 `angular_search_window = 30°`（SLAM 的 1.5 倍），使 RTCSM 的搜索空间约为 SLAM 配置的 **9 倍**。更大的搜索空间意味着更多的候选位姿需要评估。

### 4. Ceres 权重过高 → 迭代次数多

三个 Ceres 扫描匹配权重（`occupied_space_weight`、`translation_weight`、`rotation_weight`）均为 SLAM 配置的 2 倍。高权重使 Ceres 优化器需要更多迭代才能收敛，且容易陷入局部极小值。

### 恶性循环总结

```
motion_filter 激进 → 每 100ms 触发匹配（高频）
    ↓
关闭 pose_extrapolator → 无位姿预测，从零搜索
    ↓
大搜索窗口（0.3m / 30°）→ RTCSM 暴力搜索范围大
    ↓
高 Ceres 权重 → 优化迭代次数多
    ↓
CPU 30%+，且搜索范围大反而容易陷入局部极小值 → 匹配失败 → 定位丢失
```

## 修复方案

将纯定位配置完全对齐 SLAM 配置，即：

```lua
-- 位姿外推器：启用，利用运动连续性提供好的初始猜测
use_pose_extrapolator = true

-- 运动滤波：与 SLAM 一致，只有真正移动时才触发匹配
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.017

-- RTCSM 搜索窗口：与 SLAM 一致
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)

-- Ceres 权重：与 SLAM 一致
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 100
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400
```

## 修复效果

| 指标 | 修改前 | 修改后 |
|------|--------|--------|
| 点云更新频率 | 0.5~1s | 实时（~10Hz） |
| 定位精度 | 小车一动就丢失 | 稳定跟踪 |
| CPU 占用率 | >30% | <5% |

## 结论

**Cartographer 纯定位模式应与 SLAM 模式共用同一份配置文件**。两者的扫描匹配算法完全相同，参数不应有差异。纯定位的唯一区别是启动时通过 `-load_state_filename` 参数加载 .pbstream 地图文件，不再构建新地图。

建议：删除独立的 `cartographer_real_localization.lua`，纯定位模式直接使用 `cartographer_real.lua`。

## 相关文件

| 文件 | 说明 |
|------|------|
| `src/robot_slam/config/cartographer_real.lua` | SLAM 配置（基准） |
| `src/robot_slam/config/cartographer_real_localization.lua` | 纯定位配置（已对齐 SLAM） |
| `src/robot_slam/launch/slam_real_localization.launch.py` | 纯定位启动文件 |
| `docs/start_nav_real.sh` | 实车导航启动脚本 |

## 时间线

- 2026-04-23：发现纯定位效果差，开始排查
- 2026-04-23：尝试修改 search window、Ceres weights、motion filter 等参数，效果均不理想
- 2026-04-23：发现纯定位配置与 SLAM 配置存在多处差异，将纯定位完全对齐 SLAM 后问题解决
