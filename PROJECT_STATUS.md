# 项目状态

## 当前工作区

- ROS 2 工作区包含 `tf_lisense_pkg`。
- `odom_listener_node` 订阅 `/lio/imu/odom`、`/tag_pose_compensated` 和 `/local_plan_sampled`（可用 `path_topic` 修改）。位置与 yaw 保持原有低通滤波。
- 当前帧格式为 `0xAA | 23 个大端 int16 | 0xBB`，固定 48 字节。字段 0..6 为 `[odom_x_mm, odom_y_mm, odom_z_mm, odom_yaw_mrad, odom_vx_mm_s, odom_vy_mm_s, odom_vz_mm_s]`；字段 7 为有效路径点数 N（0..5）；字段 8..22 为五组 `[x_mm, y_mm, yaw_mrad]`。未使用槽位填零，N=0 表示清除路径。
- 每次里程计更新发送组合帧；路径更新也立即发送，前提是已收到有效且新鲜的里程计。默认 `path_timeout_ms=500`，路径接收超时、空路径、无效路径或无法转换坐标系时清除路径。定时器每 50 ms 检查路径超时；`odom_timeout_ms=500` 后不发送缓存里程计，下位机仍须独立设置通信超时停车。
- 路径通过 TF 按消息时间转换到里程计 `header.frame_id`；同坐标系直接使用。首次里程计之前收到的路径不缓存，等待下一条更新。超过五点、混合坐标系、无效四元数或位置超出 int16 毫米量程 [-32.768, 32.767] m 的路径整条拒绝，不截断坐标。路径接收超时按本机单调时钟计算，不代表消息生成时间有效性校验。
- AprilTag 订阅、回调及缓存保留，但不编码或发送 Tag 字段。`tag_timeout_ms` 参数暂时保留，当前发送逻辑不使用它。
- 已于 2026-10-03 通过 `colcon build --packages-select tf_lisense_pkg` 编译验证；尚未进行连接真实串口和话题的运行验证。

## 已阅读的 AprilTag 接口背景（2026-09-03）

- `apriltag_detector_cpp` 基于相机内参和 `tag_size` 解算标签 6-DOF 位姿，主要输出 `/tag_pose_compensated`：`[id, x, y, z, roll, pitch, yaw]`，位置单位 m、角度单位 deg。
- `tag_serial_bridge` 默认转发 `[id, x, y, z, yaw]`，位置乘 1000（mm）、yaw 乘 100（0.01°），以大端 `int16` 打包为 `0xAA ... 0xBB`；无标签超时后发送 `id=-1` 的丢失帧。
- 检测端可用 `publish_first_only: false` 发布同帧多个标签；桥接端可用 `target_id` 筛选某个标签。

## 后续集成注意

STM32 接收端必须改为按固定 48 字节帧读取 23 个大端有符号 16 位整数。仅使用 N 个路径点，N=0 时清除旧路径；不能将补零的槽位作为目标点。数据中也可能出现 0xAA/0xBB，须按固定帧长解析，不可遇到帧尾字节就提前结束。当前沿用旧协议，无 CRC 或确认应答。没有真实下位机验证前不能认为已完成硬件联调。
