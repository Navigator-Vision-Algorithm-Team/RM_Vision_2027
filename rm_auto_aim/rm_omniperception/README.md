# RM_Omniperception — All-Directional Perception Module

Ported from `sp_vision_25/tasks/omniperception/`. Provides multi-camera parallel detection for sentry robots, enabling 360-degree enemy awareness.

## Architecture

```
Omni-directional Cameras (up to 4)
    |   /omni_camera_0/image_raw  ...  /omni_camera_3/image_raw
    v
OmniPerceptronNode (this package)
    |
    ├── [Thread 0..N] inferLoop()
    │     camera frame → Detector::detect() → Decider::delta_angle()
    │     → push DetectionResult to ThreadSafeQueue
    |
    ├── [Main Thread] processLoop()
    │     drain queue → Decider::sort() → publish best target
    |
    └── Subscriptions:
          /omni/invincible_ids  (from referee system)
          /omni/aim_target_ids  (from navigation)
```

## Files

| File | Purpose |
|------|---------|
| `include/rm_omniperception/detection_result.hpp` | `DetectionResult` struct — armor list + camera angle offsets + timestamp |
| `include/rm_omniperception/decider.hpp` | `Decider` class — priority assignment, filtering, fallback logic |
| `include/rm_omniperception/thread_safe_queue.hpp` | `ThreadSafeQueue<T>` — bounded thread-safe queue |
| `include/rm_omniperception/omni_perceptron_node.hpp` | `OmniPerceptronNode` — ROS2 composable node, main entry point |
| `src/decider.cpp` | Decider implementation |
| `src/omni_perceptron_node.cpp` | Node + inference loop implementation |

## ROS2 Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/omni_camera_0/image_raw` | `sensor_msgs/Image` | Omni camera 0 (configurable via `camera_topic_0` param) |
| `/omni_camera_1/image_raw` | `sensor_msgs/Image` | Omni camera 1 |
| `/omni_camera_2/image_raw` | `sensor_msgs/Image` | Omni camera 2 |
| `/omni_camera_3/image_raw` | `sensor_msgs/Image` | Omni camera 3 |
| `/omni_camera_0/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics for camera 0 |
| `/omni/invincible_ids` | `std_msgs/Int32MultiArray` | Invincible enemy robot IDs (from referee) |
| `/omni/aim_target_ids` | `std_msgs/Int32MultiArray` | Priority target IDs (from navigation) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/omni/target_direction` | `geometry_msgs/Vector3Stamped` | Best omni-camera target: x=delta_yaw(rad), y=delta_pitch(rad), z=confidence |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_omni_cameras` | 4 | Number of omni-directional cameras (1-4) |
| `enemy_color` | 0 | 0=RED, 1=BLUE |
| `priority_mode` | 1 | 1=mode1 (infantry priority), 2=mode2 (engineer priority) |
| `fov_h` | 1.047 | Horizontal FOV in radians (60 degrees) |
| `fov_v` | 0.785 | Vertical FOV in radians (45 degrees) |
| `process_rate` | 50.0 | Main loop processing rate (Hz) |
| `camera_label_0..3` | `left_0`, `right_0`, `left_1`, `right_1` | Camera labels for angle offset calculation |
| `camera_topic_0..3` | `/omni_camera_0..3` | Base topic names for each camera |

## How to Enable (Future)

### Step 1: Ensure omni-camera drivers are running

Each USB camera needs a ROS2 driver publishing images. Use `usb_cam` or `hik_camera`:

```bash
# Example: launch 4 USB cameras
ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r /image_raw:=/omni_camera_0/image_raw \
  -p video_device:=/dev/video0

ros2 run usb_cam usb_cam_node_exe --ros-args \
  -r /image_raw:=/omni_camera_1/image_raw \
  -p video_device:=/dev/video2
# ... etc for video4, video6
```

### Step 2: Add to vision_bringup.launch.py

Add the omni_perceptron node to the launch file:

```python
from launch_ros.actions import ComposableNode

omni_node = ComposableNode(
    package='rm_omniperception',
    plugin='rm_omniperception::OmniPerceptronNode',
    name='omni_perceptron',
    parameters=[node_params],
    extra_arguments=[{'use_intra_process_comms': True}],
)

# Add to ComposableNodeContainer or as standalone Node
```

Or as a standalone Node:

```python
omni_node = Node(
    package='rm_omniperception',
    executable='omni_perceptron_node',
    name='omni_perceptron',
    parameters=[node_params],
)
```

### Step 3: Add to node_params.yaml

```yaml
/omni_perceptron:
  ros__parameters:
    num_omni_cameras: 4
    enemy_color: 0
    priority_mode: 1
    fov_h: 1.047
    fov_v: 0.785
    process_rate: 50.0
```

### Step 4: Integrate with tracker

The tracker can use omni-perception results to switch targets when a higher-priority enemy is detected by side cameras. The integration point is in `armor_tracker/tracker_node.cpp`:

```cpp
// Subscribe to omni target direction
omni_target_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
  "/omni/target_direction", 10,
  [this](geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
    // If tracker is in TRACKING state and omni detects higher priority:
    //   tracker_state = SWITCHING
    //   use msg->vector.x/y as gimbal delta to rotate toward new target
  });
```

### Step 5: Build

```bash
colcon build --packages-select rm_omniperception
```

## Priority Modes

### Mode 1 (default): Infantry priority
- Priority 1: Infantry 3, 4
- Priority 2: Hero (1)
- Priority 3: Infantry 5, Sentry
- Priority 4: Engineer (2)
- Priority 5: Outpost, Base, Guard

### Mode 2: Engineer priority
- Priority 1: Engineer (2)
- Priority 2: Hero (1), Infantry 3, 4, 5
- Priority 3: Sentry, Outpost, Base, Guard

## Dependencies

- `armor_detector` — uses `rm_auto_aim::Armor`, `rm_auto_aim::Detector`
- OpenCV — for image handling
- Eigen3 — for angle calculation
- rclcpp, rclcpp_components, sensor_msgs, geometry_msgs, std_msgs, cv_bridge
