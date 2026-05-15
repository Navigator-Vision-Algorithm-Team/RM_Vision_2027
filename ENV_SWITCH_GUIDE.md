# Environment Switch Guide: ROS2 Humble ↔ Jazzy

This document describes how the codebase auto-adapts between:

- **Ubuntu 22.04 + ROS2 Humble** (robot deployment)
- **Ubuntu 24.04 + ROS2 Jazzy** (development machine)

**No manual code changes are needed.** The build system auto-detects the ROS distro.

---

## How Auto-Detection Works

1. **CMake** reads the `ROS_DISTRO` environment variable (set when you source the ROS2 setup file)
2. A preprocessor define is passed to the compiler:
   - Humble → `-DROS_DISTRO_HUMBLE`, C++14
   - Jazzy → `-DROS_DISTRO_JAZZY`, C++17
3. **Source code** uses `#ifdef ROS_DISTRO_HUMBLE` / `#else` / `#endif` to compile the right variant

The detection logic (identical in all 5 CMakeLists.txt files):

```cmake
if(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
else()
  set(ROS_DISTRO "humble")   # default when not sourced
endif()
if(ROS_DISTRO STREQUAL "humble")
  set(CMAKE_CXX_STANDARD 14)
  add_definitions(-DROS_DISTRO_HUMBLE)
else()
  set(CMAKE_CXX_STANDARD 17)
  add_definitions(-DROS_DISTRO_JAZZY)
endif()
```

---

## Build Commands (Both Distros)

```bash
# Source your ROS2 environment first, then:
cd RM_Vision_2026
colcon build --symlink-install
source install/setup.bash

# Faster build (skip tests):
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

---

## Distro-Specific `#ifdef` Locations

These are the source files that branch on `ROS_DISTRO_HUMBLE`:

| File | What changes |
|------|-------------|
| `rm_auto_aim/armor_detector/src/detector_node.cpp` | `cv_bridge.h` (Humble) vs `cv_bridge.hpp` (Jazzy) |
| `rm_serial_driver/tools/aimer.hpp` | `OptionalDouble` struct (Humble) vs `std::optional<double>` (Jazzy) |
| `rm_serial_driver/tools/aimer.cpp` | `.value` member access (Humble) vs `.value()` method (Jazzy) |
| `rm_serial_driver/tools/planner/tinympc/admm.cpp` | `__attribute__((unused))` (Humble) vs `[[maybe_unused]]` (Jazzy) |

---

## Dependencies

### `serial_driver` (ROS2 package)

| Distro | Install |
|--------|---------|
| Humble | `sudo apt install ros-humble-serial-driver` |
| Jazzy | Not available via apt. Build from source: https://github.com/ros-drivers/transport_drivers |

### Python packages (Jazzy only)

```bash
pip3 install --user lark-parser empy==3.3.4
```

Not needed on Humble (no Rust IDL generator).

### OpenVINO (optional, both distros)

```bash
apt install openvino-2024.6.0
```

If not found, YOLO detector is disabled at compile time.

### System packages (both distros)

```bash
sudo apt install -y \
  libopencv-dev libeigen3-dev \
  ros-${ROS_DISTRO}-angles \
  ros-${ROS_DISTRO}-tf2 ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-tf2-geometry-msgs \
  ros-${ROS_DISTRO}-message-filters ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-image-transport ros-${ROS_DISTRO}-std-srvs \
  ros-${ROS_DISTRO}-nav2-msgs
```

---

## Other Distro Differences (No Code Impact)

| Aspect | Humble | Jazzy |
|--------|--------|-------|
| Python | 3.10 | 3.12/3.13 |
| GCC | 11.x | 13.x |
| `ament_cmake_auto` scoped headers | No warning | Warning only (cosmetic) |
| `M_PI` / `CV_PI` | Available | Available |
| `ament_index_cpp` header path | Same | Same |
