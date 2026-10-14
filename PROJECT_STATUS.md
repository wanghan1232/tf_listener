# 项目状态

## 当前工作区

- ROS 2 工作区包含 `tf_lisense_pkg`。
- `odom_listener_node` 订阅 `/lio/imu/odom` 与 `/tag_pose_compensated`。前者触发发送频率，并对位置与 yaw 做一阶低通滤波。
- 当前组合帧格式为 `0xAA | 12 个大端 int16 | 0xBB`（26 字节）：`[odom_x_mm, odom_y_mm, odom_z_mm, odom_yaw_mrad, odom_vx_mm_s, odom_vy_mm_s, odom_vz_mm_s, tag_id, tag_x_mm, tag_y_mm, tag_z_mm, tag_yaw_0.01deg]`。
- Tag 数据缓存超过 `tag_timeout_ms`（默认 500 ms）未更新时，Tag 的 5 个字段发送 `[-1, 0, 0, 0, 0]`；整帧仍按里程计频率发送。
- 已于 2026-10-03 通过 `colcon build --packages-select tf_lisense_pkg` 编译验证；尚未进行连接真实串口和话题的运行验证。

## 已阅读的 AprilTag 接口背景（2026-09-03）

- `apriltag_detector_cpp` 基于相机内参和 `tag_size` 解算标签 6-DOF 位姿，主要输出 `/tag_pose_compensated`：`[id, x, y, z, roll, pitch, yaw]`，位置单位 m、角度单位 deg。
- `tag_serial_bridge` 默认转发 `[id, x, y, z, yaw]`，位置乘 1000（mm）、yaw 乘 100（0.01°），以大端 `int16` 打包为 `0xAA ... 0xBB`；无标签超时后发送 `id=-1` 的丢失帧。
- 检测端可用 `publish_first_only: false` 发布同帧多个标签；桥接端可用 `target_id` 筛选某个标签。

## 后续集成注意

AprilTag 默认串口载荷为 5 个 word（含 `id`），里程计载荷为 7 个 word（无 `id`），且 yaw 定标单位不同。本工作区已将两者合并为一帧 12 个 word；STM32 必须按上述字段顺序读取。
