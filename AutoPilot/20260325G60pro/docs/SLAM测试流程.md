# G60Pro 自动驾驶仿真测试完整流程

## 准备工作

在开始前，确保所有残留进程已清理：
```bash
pkill -9 -f gzserver
pkill -9 -f gazebo
pkill -9 -f spawn
pkill -9 -f cartographer
sleep 2
```

---

## 完整操作流程

### 第1步：编译项目

```bash
cd /home/admin123/Development/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build
```

---

### 第2步：启动Gazebo仿真 (终端1)

```bash
cd /home/admin123/Development/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_gazebo sim.launch.py
```

等待机器人模型加载完成（看到 "Successfully spawned entity [g60pro]"）。

---

### 第3步：启动Cartographer SLAM (终端2)

> **注**：底盘节点不需要启动。Gazebo 的 `libgazebo_ros_planar_move.so` 插件已提供 `/odom` 话题和 `odom → base_footprint` TF。

```bash
cd /home/admin123/Development/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_slam slam.launch.py use_sim_time:=true
```

等待看到 "Added trajectory with ID '0'" 和 "Inserted submap (0, 0)"。

---

### 第4步：启动RViz可视化 (终端3)

```bash
cd /home/admin123/Development/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2
```

在RViz中添加显示项：
- `Map` → 话题: `/map`
- `RobotModel` → 显示机器人模型
- `LaserScan` → 话题: `/scan` (单线激光，用于Nav2导航)
- `PointCloud2` → 话题: `/points2` (多线激光，用于建图)

---

### 第5步：键盘控制机器人移动 (终端4)

```bash
cd /home/admin123/Development/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/cmd_vel
```

键盘控制说明：
- `i` - 前进
- `,` - 后退
- `j` - 左转
- `l` - 右转
- `k` - 停止

控制机器人环绕房间移动，让Cartographer完成建图。

---

### 第6步：保存地图 (可选)

建图完成后，在新终端执行：

```bash
cd /home/admin123/Development/AutoPilot/20260325G60pro
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/robot_bringup/maps/g60pro_map
```

这将生成 `g60pro_map.yaml` 和 `g60pro_map.pgm` 文件。

---

## 注意事项

1. **不要按 Ctrl+C** - 在建图过程中保持所有终端运行
2. 如果Gazebo崩溃，使用准备工作的清理命令后再重新启动
3. 确保四个终端同时运行以完成完整的SLAM测试：
   - 终端1：Gazebo仿真
   - 终端2：Cartographer SLAM
   - 终端3：RViz可视化
   - 终端4：键盘控制

---

## 进入Navigation阶段

SLAM测试完成后，可进入Navigation阶段：
- 使用单线激光雷达 (`/scan`) + RGBD相机进行导航
- 需要创建 `robot_navigation` 包的配置文件
- 加载保存的地图进行自主导航