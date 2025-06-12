# trajectory_saver

This packages is used for saving trajectory for the manipulation system. It will save a String message with the trajectory data in MongoDB. The package creates the services to manage the database.

The package includes a launch that will start the mongodb comunication with the database, the message_store_node and the trajectory_saver_node.

1. The first launch (mongodb_store/launch/mongodb_store.launch) starts the comunication with the MongoDB database. The port and path to the database must be set. Moveit launches also comunication with the database (default_warehouse_mongo_db) which is a MongoDB database.

2. message_store_node starts all the configuration for saving ROS messages.

3. trajectory_saver_node is the main node of this package. It creates the services to manage the data in the database. It will structure all the possible expansions of using the database (at the moment we are saving std_msgs::String but can save any type of message even custom messages).

## How to use the package?

First, launch the trajectory_database.launch file with the correct configuration:

```
roslaunch trajectory_saver trajectory_database.launch database_port:=63222 database_path:=$(find trajectory_saver/default_warehouse_mongo_db)
```

Then, the services are ready to be used:

- /trajectory_saver/add_trajectory
- /trajectory_saver/get_trajectory
- /trajectory_saver/remove_trajectory
- /trajectory_saver/amount_of_trajectories
- /trajectory_saver/add_pose
- /trajectory_saver/get_pose
- /trajectory_saver/remove_pose
- /trajectory_saver/list_poses_names

### /trajectory_saver/add_trajectory

Type: trajectory_saver_msg/AddTrajectory

Args:

- name: string with the name to sev the trajectory
- trajectory: string with the trajectory data

It will return true if the message has been added or updated and the message returned will be "Added" or "Updated".


```
rosservice call /trajectory_saver/add_trajectory "name: 'Prueba'
trajectory: 'primera tayectoria'"

success: True
message: "Added"
```

```
rosservice call /trajectory_saver/add_trajectory "name: 'Prueba'
trajectory: 'segunda tayectoria'"

success: True
message: "Updated"
```

### /trajectory_saver/get_trajectory

Type: trajectory_saver_msg/GetTrajectory

Args:

- name: string with the name of the trajectory

It will return:

- success: true if it exist, false if not or any other issue
- trajectory: the trajectory string saved


```
rosservice call /trajectory_saver/get_trajectory "name: 'Prueba'"

success: True
trajectory: "segunda tayectoria"

```

```
rosservice call /trajectory_saver/get_trajectory "name: 'Prueba'" 
success: False
trajectory: "NOT EXISTS"
```

### /trajectory_saver/remove_trajectory

Type: trajectory_saver_msg/RemoveTrajectory

Args:

- name: string with the name of the trajectory

It will return:

- success: true if it was succesfully removed or false if there is any error or the trajectory doesn't exist


```
rosservice call /trajectory_saver/remove_trajectory "name: 'Prueba'"

success: True
```

### /trajectory_saver/amount_of_trajectories

Type: std_srvs/Trigger

Args:

It will return 2 variables:

- success: true if there is any trajectory saved or false if there are no trajectories or any error with the database
- message: number of trajectories

```
rosservice call /trajectory_saver/amount_of_trajectories "{}"

success: True
message: "1"
```

```
rosservice call /trajectory_saver/amount_of_trajectories "{}"

success: False
message: "0"
```


### /trajectory_saver/add_pose

Type: trajectory_saver_msg/AddPose

Args:

- name: string with the name to sev the pose
- tf_pose - pose with quaternion rotation
- rpy_pose - Roll Pitch Yaw pose
- joint_pose - std joint state msgs type

It will return true if the message has been added or updated and the message returned will be "Added" or "Updated".


```
rosservice call /trajectory_saver/add_pose "name: ''
tf_pose:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
rpy_pose: {x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}
joint_pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  name: ['']
  position: [0]
  velocity: [0]
  effort: [0]"

success: True
message: "Added"
```

```
rosservice call /trajectory_saver/add_pose "name: ''
tf_pose:
  translation:
    x: 0.0
    y: 0.0
    z: 0.0
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0
rpy_pose: {x: 0.0, y: 0.0, z: 0.0, roll: 0.0, pitch: 0.0, yaw: 0.0}
joint_pose:
  header:
    seq: 0
    stamp: {secs: 0, nsecs: 0}
    frame_id: ''
  name: ['']
  position: [0]
  velocity: [0]
  effort: [0]"

success: True
message: "Updated"
```


### /trajectory_saver/get_pose

Type: trajectory_saver_msg/GetPose

Args:

- name: string with the name of the pose

It will return:

- success: true if it exist, false if not or any other issue
- tf_pose
- rpy_pose
- joint_pose


```
rosservice call /trajectory_saver/get_pose "name: 'Prueba'"

success: True
message: ''
tf_pose: 
  translation: 
    x: -0.2094543615378314
    y: -0.3149658117572486
    z: 0.34701533774632015
  rotation: 
    x: 0.7097973032721033
    y: -0.0835733323604105
    z: 0.6967071015842098
    w: 0.06166442238402706
rpy_pose: 
  x: -0.35844144225120544
  y: 0.12079858779907227
  z: 0.3470079004764557
  roll: 1.2535295486450195
  pitch: -1.2197319269180298
  yaw: 1.2608076333999634
joint_pose: 
  header: 
    seq: 90521
    stamp: 
      secs: 1746434808
      nsecs: 304687153
    frame_id: ''
  name: 
    - robot_arm_elbow_joint
    - robot_arm_shoulder_lift_joint
    - robot_arm_shoulder_pan_joint
    - robot_arm_wrist_1_joint
    - robot_arm_wrist_2_joint
    - robot_arm_wrist_3_joint
  position: [-2.7631566524505615, -1.2988131505301972, -3.142798725758688, -0.6786829394153138, 1.5495738983154297, -1.628315273915426]
  velocity: [-0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  effort: [9.444382667541504, 0.5233883857727051, -1.085978627204895, 2.9762492179870605, 0.15563079714775085, 0.441201776266098]
```

```
rosservice call /trajectory_saver/get_pose "name: 'Prueba'" 
success: False
message: "TF Pose does not exist"
```

### /trajectory_saver/remove_pose

Type: trajectory_saver_msg/RemovePose

Args:

- name: string with the name of the pose

It will return:

- success: true if it was succesfully removed or false if there is any error or the pose doesn't exist


```
rosservice call /trajectory_saver/remove_pose"name: 'Prueba'"

success: True
```

### /trajectory_saver/list_poses_names

Type: trajectory_saver_msg/GetListOfPoses

Args:

- none

It will return:

- success: true if it found more than one pose in the list
- names: list of names f poses available in database

```
rosservice call /trajectory_saver/list_poses_names

success: True
message: ''
names: 
  - HOME
  - READY
```


All the [custom srv are in this package](https://github.com/RobotnikAutomation/trajectory_saver_msg).



