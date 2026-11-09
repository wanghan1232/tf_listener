# 项目状态

## 当前工作区

- ROS 2 工作区包含 `tf_lisense_pkg`。
- `odom_listener_node` 订阅 `/lio/imu/odom`、`/tag_pose_compensated` 和 `/cmd_vel_nav`（可用 `cmd_vel_topic` 修改）。位置与 yaw 保持原有低通滤波。
- 当前帧格式为 `0xAA | 10 个大端 int16 | 0xBB`，固定 22 字节。字段 0..6 保留 `[odom_x_mm, odom_y_mm, odom_z_mm, odom_yaw_mrad, odom_vx_mm_s, odom_vy_mm_s, odom_vz_mm_s]`；字段 7..9 为 DWB `[linear_x_mm_s, linear_y_mm_s, angular_z_mrad_s]`。终端周期打印三项 DWB 速度，单位分别为 m/s、m/s、rad/s。
- 每次里程计更新发送组合帧，帧尾包含最新收到的 DWB 速度；DWB 速度尚未收到时按零发送。下位机仍须独立设置通信超时停车。
- AprilTag 订阅、回调及缓存保留，但不编码或发送 Tag 字段。`tag_timeout_ms` 参数暂时保留，当前发送逻辑不使用它。
- 已于 2026-10-03 通过 `colcon build --packages-select tf_lisense_pkg` 编译验证；尚未进行连接真实串口和话题的运行验证。

## 已阅读的 AprilTag 接口背景（2026-09-03）

- `apriltag_detector_cpp` 基于相机内参和 `tag_size` 解算标签 6-DOF 位姿，主要输出 `/tag_pose_compensated`：`[id, x, y, z, roll, pitch, yaw]`，位置单位 m、角度单位 deg。
- `tag_serial_bridge` 默认转发 `[id, x, y, z, yaw]`，位置乘 1000（mm）、yaw 乘 100（0.01°），以大端 `int16` 打包为 `0xAA ... 0xBB`；无标签超时后发送 `id=-1` 的丢失帧。
- 检测端可用 `publish_first_only: false` 发布同帧多个标签；桥接端可用 `target_id` 筛选某个标签。

## 后续集成注意

STM32 接收端需按固定 22 字节帧读取 10 个大端有符号 16 位整数。字段 0..6 保持不变，字段 7..9 新增 DWB 的 x/y 线速度和 z 角速度。数据中也可能出现 0xAA/0xBB，须按固定帧长解析，不可遇到帧尾字节就提前结束。当前沿用旧协议，无 CRC 或确认应答；需同步更新下位机解析，尚未完成真实下位机联调。
