# orbbec_description

**Orbbec 相机 URDF 模型**

## 功能

提供 Orbbec 相机的 URDF 模型描述，用于 RViz 可视化和 TF 树。

## 模型文件

### `urdf/dabai_dcw2.urdf.xacro`

**包含内容**：
- 相机本体 link
- 彩色相机 optical frame
- 深度相机 optical frame
- 红外相机 optical frame

**TF 树结构**：
```
camera_link
  ├─ camera_color_frame
  │    └─ camera_color_optical_frame
  ├─ camera_depth_frame
  │    └─ camera_depth_optical_frame
  └─ camera_ir_frame
       └─ camera_ir_optical_frame
```

---

## 使用方法

### 集成到机器人模型

在 `robot_description/urdf/g60pro.urdf.xacro` 中引用：

```xml
<xacro:include filename="$(find orbbec_description)/urdf/dabai_dcw2.urdf.xacro"/>

<!-- 前相机 -->
<xacro:dabai_dcw2 
  parent="base_link" 
  name="camera_front"
  xyz="0.75 0 0.5" 
  rpy="0 0 0"/>
```

---

## 启动文件

### `launch/view_model.launch.py`

**功能**：在 RViz 中查看相机模型

**启动命令**：
```bash
ros2 launch orbbec_description view_model.launch.py
```

---

## 依赖

- `xacro` — URDF 宏处理
- `robot_state_publisher` — 发布 TF 树
- `rviz2` — 可视化

---

## 注意事项

1. **Optical Frame**：相机 optical frame 遵循 ROS 相机坐标系约定（Z 轴向前，X 轴向右，Y 轴向下）
2. **TF 树**：确保相机 link 正确连接到机器人 TF 树
3. **多相机**：每个相机需要唯一的 `name` 参数，避免 TF 冲突
