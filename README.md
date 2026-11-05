# trajectory_saver

## Purpose and Relevance

Persistent named trajectory-string storage for ROS 2 Jazzy using the actual
`warehouse_ros_sqlite` plugin. Relevant to application-defined reusable
trajectory data. The node does not plan, parse, validate or execute strings.
Pose storage belongs to the independent [pose_saver](../pose_saver/README.md)
package; this node exposes no pose services and opens no pose collection.

The executable and default node name are `trajectory_saver_node`.

## Build and Test

On Ubuntu 24.04 with ROS 2 Jazzy installed:

```bash
sudo apt install ros-jazzy-warehouse-ros-sqlite ros-jazzy-moveit-msgs \
  ros-jazzy-ament-cmake-gtest
source /opt/ros/jazzy/setup.bash
cd /home/lgranat/workspaces/hmi_centauro_ws
colcon build --base-paths src/standard_manipulation_packages \
  --packages-up-to trajectory_saver
source install/setup.bash
colcon test --base-paths src/standard_manipulation_packages \
  --packages-select trajectory_saver
colcon test-result --verbose
```

This builds all local planning, manipulation and storage interfaces in
`manipulation_msgs` as well.
Restrict discovery to the migrated tree to avoid duplicate names from the
adjacent ROS 1 packages. For both storage nodes and their integration test,
use [database_pose_manager's stack build](../database_pose_manager/README.md#build-and-test).
Tests use temporary databases, not the configured operational file.

## Launch and Configuration

[trajectory_saver.launch.xml](launch/trajectory_saver.launch.xml) starts only
the saver. Its arguments are `namespace`, `warehouse_host` and `database_name`:

```bash
ros2 launch trajectory_saver trajectory_saver.launch.xml \
  namespace:=/robot/arm warehouse_host:=$HOME/.ros/manipulation.sqlite \
  database_name:=manipulation
ros2 launch trajectory_saver trajectory_saver.launch.xml --show-args
```

The launch loads the installed YAML first, then applies explicit argument
overrides. Use the `ros2 run --params-file` example below for direct node
configuration. The manager's `pose_storage.launch.xml` starts pose storage
only, not this node. Start trajectory storage separately when needed. Do not
run two trajectory writers for the same collection.

## YAML Parameter Reference

The installed defaults are in
[config/trajectory_saver.yaml](config/trajectory_saver.yaml). The `/**`
selector matches the node in any namespace; pass the file only to the saver.

| Parameter | YAML/default value | Explanation |
| --- | --- | --- |
| `warehouse_plugin` | `warehouse_ros_sqlite::DatabaseConnection` | Plugin loaded through `warehouse_ros`; requires the Jazzy SQLite package |
| `warehouse_port` | `0` | Compatibility parameter; unused by the SQLite backend |
| `database_name` | `manipulation` | Nonempty logical warehouse database, distinct from the physical file |
| `warehouse_host` | Omitted; runtime default `$HOME/.ros/manipulation.sqlite` | SQLite file path, not a network hostname |

`warehouse_host` is intentionally omitted from the shipped YAML so the node
computes the user's home-directory default at runtime. YAML does not expand
`$HOME`, `~` or launch substitutions inside arbitrary parameter strings.
For another location, use an absolute path in a custom file:

```yaml
/**:
  ros__parameters:
    warehouse_plugin: warehouse_ros_sqlite::DatabaseConnection
    warehouse_host: /var/lib/robot/manipulation.sqlite
    warehouse_port: 0
    database_name: manipulation
```

The process needs permission to create/write that location. Parent directories
are created automatically. Empty paths/database names, connection failures
and stored-message schema mismatches fail startup. A relative path depends
on the process working directory and is discouraged. These are startup
settings; restart rather than relying on runtime parameter changes.

Run a YAML-configured saver alone:

```bash
ros2 run trajectory_saver trajectory_saver_node --ros-args \
  -r __ns:=/robot/arm \
  --params-file "$(ros2 pkg prefix trajectory_saver)/share/trajectory_saver/config/trajectory_saver.yaml" \
  -p warehouse_host:=$HOME/.ros/manipulation.sqlite
```

## Service API

Every endpoint is fixed under `/robot/arm/trajectory_saver/`, independent of
the node's launch namespace and name.

| Suffix | Type | Behavior |
| --- | --- | --- |
| `add_trajectory` | `manipulation_msgs/srv/AddTrajectory` | Add or replace an opaque string |
| `get_trajectory` | `manipulation_msgs/srv/GetTrajectory` | Retrieve that exact string |
| `remove_trajectory` | `manipulation_msgs/srv/RemoveTrajectory` | Remove every revision of the named trajectory |
| `amount_of_trajectories` | `std_srvs/srv/Trigger` | Decimal unique-name count in `message` |

Names must not be empty/whitespace-only. Successful empty counts have `success: true`;
missing get/remove records return `success: false`. Storage exceptions return
failure and error text when the response has `message`; errors for
`GetTrajectory` and `RemoveTrajectory` are logged because those contracts
have no error-message field. Always inspect `success`.

## Interaction Examples

Store an application-defined trajectory string without commanding motion:

```bash
ros2 service call /robot/arm/trajectory_saver/add_trajectory \
  manipulation_msgs/srv/AddTrajectory \
  '{name: inspection_path, trajectory: "application-defined trajectory data"}'
ros2 service call /robot/arm/trajectory_saver/get_trajectory \
  manipulation_msgs/srv/GetTrajectory '{name: inspection_path}'
ros2 service call /robot/arm/trajectory_saver/amount_of_trajectories \
  std_srvs/srv/Trigger '{}'
ros2 service call /robot/arm/trajectory_saver/remove_trajectory \
  manipulation_msgs/srv/RemoveTrajectory '{name: inspection_path}'
ros2 param dump /robot/arm/trajectory_saver_node
```

Calling `add_trajectory` again with the same name updates the string. The
application defines the string format and any parsing, validation or execution.
This is not a typed `moveit_msgs/RobotTrajectory` storage API.

## Database Layout and Persistence

This node opens only the `trajectories` collection:

| Collection | Message type | Metadata |
| --- | --- | --- |
| `trajectories` | `std_msgs/msg/String` | `name`, integer `revision` |

Replacements insert a newer revision before deleting older ones; failed insertion preserves
the previous record. Cleanup failure leaves older revisions, which reads
ignore by selecting the newest and counts deduplicate. Replacement and
cleanup are not one database transaction. Normal replacement does not retain
an intentional history. The file persists after shutdown and can be reopened
by the node or another compatible warehouse client.

The separate pose saver opens only `poses`. Both nodes may share the same
SQLite file and logical database, with one writer per collection. Their
defaults preserve the previous database layout. Use separate files for heavy
concurrent workloads. Pose and trajectory names may coincide without
interference. A pose schema problem does not prevent this node from opening
its trajectory collection.

These are application collections, not MoveIt planning-scene/trajectory
collections. They do not automatically appear in MoveIt's warehouse UI.
No ROS 1 MongoDB data or serialized messages are automatically migrated.
For backup, stop writers before copying the SQLite database and account for
any SQLite sidecar files; use SQLite's backup facilities for a live database.

## Relationship to Other Packages

- [manipulation_msgs](../manipulation_msgs/README.md#storage-interfaces) defines
  the trajectory CRUD services; `std_msgs/String` is the persisted message.
- `warehouse_ros` provides the C++ collection/plugin API;
  `warehouse_ros_sqlite` provides the actual persistent backend.
- [pose_saver](../pose_saver/README.md) stores poses independently;
  [database_pose_manager](../database_pose_manager/README.md) uses that node,
  not trajectory storage. This saver requires no robot driver or TF.
- HMI and task clients may query/modify the catalog independently. External
  motion packages must validate and execute retrieved data themselves.

## Tests

After building the complete stack and sourcing it, run from the workspace root:

```bash
colcon test --base-paths src/standard_manipulation_packages \
  --packages-select pose_saver trajectory_saver database_pose_manager
colcon test-result --verbose
```

The saver test uses the real plugin and a temporary SQLite file to verify
CRUD, replacements, invalid names and persistence after reopen.
The manager's three-node test exercises all storage services, restarts each
saver independently and checks concurrent writes to separate collections.

## ROS 1 Origin and Migration

The original Python package at
`standard_manipulation_packages_original/utilities/trajectory_saver/trajectory_saver`
uses MongoDB's message store. Each pose occupies three separately named
records ending in `_tf`, `_rpy` and `_joint`; partial writes or deletes can
leave incomplete poses. Its `get_trajectory` service is mistakenly wired to
the pose callback. Trajectories are opaque strings in both versions, not
MoveIt `RobotTrajectory` messages.

The Jazzy port fixes that callback wiring and replaces MongoDB with the
installed SQLite warehouse plugin. Pose handling is now in `pose_saver`.
Existing ROS 1
MongoDB records, ROS 1 serialized messages and MoveIt warehouse collections
are not converted automatically. Export old data and reinsert it through
the ROS 2 services when migration is required; use the application's intended
pose frames and trajectory-string format during conversion.

Other warehouse clients can access the same database and typed collections,
but this storage layout is not MoveIt's warehouse UI layout. See
[Database Layout and Persistence](#database-layout-and-persistence) for
replacement semantics and schema constraints. Licensing metadata retains
the original `TODO`; establish the source license before redistribution.

The runtime split leaves trajectory endpoints, types, collection and metadata
unchanged, so existing trajectory records need no conversion. Pose endpoints
have moved to `pose_saver`; see [pose migration](../pose_saver/README.md#origin-and-migration).
The previous consolidation of interface packages into `manipulation_msgs`
is separate from this runtime split; its compatibility notes remain in
[manipulation_msgs](../manipulation_msgs/README.md#compatibility).