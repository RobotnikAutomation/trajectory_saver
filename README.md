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


All the [custom srv are in this package](https://github.com/RobotnikAutomation/trajectory_saver_msg).



