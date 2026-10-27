# TrailerLoadingPlacement

## Perception result interface

[`free_space_detection/msg/PerceptionResult`](packages/perception/msg/PerceptionResult.msg)
contains the perception lifecycle state and its result:

| `status_code` | Meaning |
|---|---|
| `STATUS_IDLE` (0) | No perception operation is in progress. |
| `STATUS_PROCESSING` (1) | Perception is running; no final result is available. |
| `STATUS_SUCCEEDED` (2) | Perception completed successfully. |
| `STATUS_FAILED` (3) | Perception completed unsuccessfully. |

`success` must be true if and only if `status_code` is `STATUS_SUCCEEDED`.
All geometry fields are valid only when `success` is true:

- `header.stamp` is the input point cloud acquisition time.
- `header.frame_id` is the common coordinate frame for `trailer_pose` and
  `slot_position`; positions and dimensions are in meters.
- `trailer_dimensions.x/y/z` are length, width, and height along trailer-local axes.
- `trailer_pose` is the trailer bounding-box center and its quaternion orientation.
- `slot_position` is the center of the selected slot footprint on the trailer floor.

The interface is defined and registered for generation; the perception node does
not publish this message yet.

In C++, include the generated header and use the generated message type:

```cpp
#include "free_space_detection/msg/perception_result.hpp"

using PerceptionResult = free_space_detection::msg::PerceptionResult;
```

The node target links the package's generated `rosidl_typesupport_cpp` target to
obtain generated include paths, type support, and message-generation dependencies.

Build and inspect it from the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select free_space_detection
source install/setup.bash
ros2 interface show free_space_detection/msg/PerceptionResult
```

## Common QoS profiles

[`QoS.hpp`](packages/perception/include/free_space_detection/types/QoS.hpp) defines
immutable profiles in `free_space_detection::qos`. All profiles use KeepLast:

| Profile | Depth | Reliability | Durability | Typical use |
|---|---|---|---|---|
| `SENSOR_DATA` | 5 | Best effort | Volatile | Point clouds, images, visualization |
| `SENSOR_LATEST` | 1 | Best effort | Volatile | Latest sensor frame; discard queued older frames |
| `RELIABLE` | 10 | Reliable | Volatile | General low-rate topic communication |
| `RELIABLE_LATEST` | 1 | Reliable | Volatile | Latest perception result or pose |
| `LATCHED_STATE` | 1 | Reliable | Transient local | Last state for late-joining subscribers |

```cpp
#include "free_space_detection/types/QoS.hpp"

mResultPub = create_publisher<free_space_detection::msg::PerceptionResult>(
    "detect_result", free_space_detection::qos::RELIABLE_LATEST);
```

For historical delivery with `LATCHED_STATE`, subscribers must also request
transient-local durability. The retained state normally lasts only while the
publisher is alive; validate its timestamp and task identity before using it.
A best-effort publisher cannot satisfy a reliable subscriber, and a volatile
publisher cannot satisfy a transient-local subscriber.

Reliable delivery and bounded history do not guarantee that the application
processes every task result. Use task acknowledgements or an action when needed.
Copy a profile before customizing it so the shared defaults remain unchanged:

```cpp
auto imu_qos = free_space_detection::qos::SENSOR_DATA;
imu_qos.keep_last(2000);
```
