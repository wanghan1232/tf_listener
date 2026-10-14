# Repository Guidelines

## Project Structure & Module Organization

This source workspace contains one ROS 2 package, `tf_lisense_pkg/`.

- `tf_lisense_pkg/src/odom_listener_node.cpp` implements the `odom_listener_node` ROS 2 executable.
- `tf_lisense_pkg/include/tf_lisense_pkg/serial_sender.hpp` provides the Linux `termios` serial-port and frame-encoding helpers.
- `tf_lisense_pkg/CMakeLists.txt` declares the C++17 build, dependencies, and install targets.
- `tf_lisense_pkg/package.xml` is the package manifest; keep CMake and manifest dependencies aligned.

Place implementation files in `src/` and public headers in `include/tf_lisense_pkg/`. Keep ROS topic handling in node code and reusable serial protocol logic in headers or dedicated library sources.

## Build, Test, and Development Commands

From the ROS 2 workspace root (the directory containing this `src/` directory), run:

```bash
source /opt/ros/<distro>/setup.bash
colcon build --packages-select tf_lisense_pkg
source install/setup.bash
ros2 run tf_lisense_pkg odom_listener_node
```

Use parameters when hardware differs, for example:

```bash
ros2 run tf_lisense_pkg odom_listener_node --ros-args -p serial_device:=/dev/ttyUSB0 -p serial_baudrate:=115200
```

Run `colcon test --packages-select tf_lisense_pkg` after adding tests, followed by `colcon test-result --verbose`. The package currently has no committed automated tests.

## Coding Style & Naming Conventions

Use C++17 and retain the existing four-space indentation and brace-on-next-line style. Name classes in `PascalCase` (`OdomListener`), methods in `camelCase` (`tagPoseCallback`), and member state with the established `m_` prefix. Prefer `constexpr` for constants, fixed-width integer types for protocol fields, and explicit unit conversions with saturation checks. Keep compiler warnings enabled; `CMakeLists.txt` already applies `-Wall -Wextra -Wpedantic` for GCC and Clang.

## Testing Guidelines

Add focused tests under `tf_lisense_pkg/test/` and register them with `ament_cmake`/`ament_cmake_gtest`. Name test files after their subject, such as `test_serial_sender.cpp`. Cover frame byte order, boundary saturation, and invalid or stale tag data. Hardware tests must state the required serial device and topic publishers.

## Commit & Pull Request Guidelines

Git history is unavailable in this checkout, so use short imperative commits such as `Add tag frame validation`. Keep each commit scoped. Pull requests should explain behavior changes, list ROS parameters/topics affected, include build and test results, and attach logs or screenshots when runtime behavior changes. Never commit device-specific paths, credentials, or generated `build/`, `install/`, and `log/` directories.
