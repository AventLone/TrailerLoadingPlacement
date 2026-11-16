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

## Localization executables

The localization package builds and installs two separate executables:

| Executable | Entry point | Node name |
|---|---|---|
| `localization` | [`source/localization.cpp`](packages/localization/source/localization.cpp) | `dynamic_localization` |
| `mapping` | [`source/mapping.cpp`](packages/localization/source/mapping.cpp) | `mapping` |

Each target includes its own node implementation and the shared GICP and
SparseVoxel sources, without linking both entry points into the same executable.
Build from the workspace root and start the mapping node with:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select localization
source install/setup.bash
ros2 run localization mapping
```

To start localization instead, use `ros2 run localization localization`.

## Mapping parameters

[`Mapping.h`](packages/localization/include/localization/nodes/Mapping.h)
declares all mapping parameters in `declareParameters()` before initialization.
Topic selection and intensity filtering use the startup parameter values:

| Parameter | Default | Purpose |
|---|---|---|
| `Topics.LidarScan` | `/sim_scan` | Input LiDAR point cloud |
| `Topics.RegisteredScan` | `/rigistered_scan` | Registered point cloud output |
| `Topics.CloudMap` | `/cloud_map` | Voxel map output |
| `Topics.BasePose` | `/base_pose` | Estimated pose output |
| `Topics.PosePath` | `/base_pose_path` | Pose path output |
| `intensity_threshold` | `-1.0` | Negative values select a threshold from the first scan |
| `intensity_keep_ratio` | `0.99` | Target retained fraction, clamped to `[0.01, 1.0]` |

`MapSavingPath` supplies the directory used by `saveMap()` for `map.pcd`; its
default is `/home/avent/Desktop/TrailerLoadingPlacement/packages/localization/map`.
Declaring this parameter does not trigger automatic map saving.

Override parameters at startup with ROS arguments, for example
`--ros-args -p Topics.LidarScan:=/rslidar_points -p Topics.RegisteredScan:=/scan_vis`.

[`mapping.yaml`](packages/localization/config/mapping.yaml) contains these
defaults for the `mapping` node and is installed to `share/localization/config`.
When starting the node from the workspace root, load it with
`--ros-args --params-file packages/localization/config/mapping.yaml`.

## Dynamic localization parameters

[`DynamicLocalization.h`](packages/localization/include/localization/nodes/DynamicLocalization.h)
centralizes declarations in `declareParameters()` before initialization, using
`declare_parameters` to group parameters of the same type. Its `Topics.*` keys
match those above, but `Topics.RegisteredScan` defaults to `/scan_vis`.
Subscribers and publishers use the startup topic values without changing QoS.

`intensity_threshold` defaults to `-1.0`, and `intensity_keep_ratio` defaults to
`0.99` with the existing `[0.01, 1.0]` clamp. The existing `MapSavingPath` and
`Topic.StaticCloudMap` (`/static_cloud_map`) parameters are retained, but the
dynamic node does not yet consume them for map saving or static-map input.

## Static-map cloud differencing

[`Localization::getDynamicCloud`](packages/localization/source/nodes/Localization.cpp)
expects a scan already registered in the static map's coordinate frame. It
queries the existing static-map KD-tree and retains points whose nearest static
neighbor is more than `0.1 m` away; points exactly at the threshold remain static.
Non-finite scan points are discarded with a warning. The returned cloud is
unorganized and preserves the input header and sensor pose metadata.

These points differ from the static map but are not necessarily moving objects.
The function does not modify either map or the registration target, and is not
yet connected to the worker loop.
