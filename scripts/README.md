# nxp-uwb-ros2 scripts

To run this in simulation launch the B3RB SIL simulation using the [uwb-demo branch of dream and specifying `world:=uwb_map`](https://github.com/CogniPilot/dream_world/tree/uwb-demo). 

After the simulation launches execute the `run.sh` bash script.

This bash script does the following:

- launches **gz_twr_point.py** which is used to subscribe to `/uwb_twr_source` which is a logical camera with matching FOV and range as the UWB module. The logical camera reports back anything it sees in that FOV and range by object name and it's relative image frame based distances as a vector3d. When the object it sees matches the B3RB object name (which has a UWB tag) it publishes that relative position data on `/uwb_twr_point` in GZ transport.

- launches **ros_gz_twr_data_generator.py** which subscribes the the relative position provided by `/uwb_twr_point` published from **gz_twr_point.py** and performs the mathematical operations to convert it to azimuth, elevation, distance and AoA along with a noise distribution applied. That is then converted to q9.7 and packed in the the `TwoWayRanging` message inside the `UltraWideBandRanging` ROS 2 message and sent out on `/sr1xx/twr_responder` to mimic the exact same data you would get in the real world use case.

- launches **twr_point_publisher.py** which subscribes to the `/sr1xx/twr_responder` published by **ros_gz_twr_data_generator.py** (or for real world or mixed simulations can listen to the topic from a bag file or system directly) and performs the mathematical operations to spatially place the read UWB data into a relative cartesian coordinate point. That point is then published as a `PointStamped` ROS 2 message on the topic `/point_stamped`.

- Runs **`ros2 topic pub --once /joy sensor_msgs/msg/Joy "{buttons: [0, 0, 0, 0, 0, 0, 0, 1]}"`** to initially arm the vehicle so it can begin the driving operations.

- Runs **`ros2 run ros_gz_bridge parameter_bridge /uwb_twr_point@geometry_msgs/msg/Point@gz.msgs.Vector3d`** converts the `/uwb_twr_point` from GZ transport to ROS 2.

- launches **demo_control.py** which is the main control script for the B3RB rover, this is written with self explanatory variable names and callbacks, it is beneficial to open and read this file. At it's core it uses a set of user defined points defined in `self.points` which are a set of x and y coordinates it to feeds to the `self.nav2_goal_topic` for navigation with NAV2. One of the points can also be listed as "UWB" which then means it uses the average of the last `self.max_queue_size` read transforms of the UWB anchor to navigate to that point. As it subscribes to it's current position on `self.nav2_pose_topic` it compares that to it's `self.currentGoalPose` from it's `self.currentGoalIndex` of the provided `self.points` and sees if it's within the `self.GoalAchievedThreshold`. If the B3RB is within `self.GoalAchievedThreshold` it will flash it's lights and iterate/loop through to the next `self.points`. There is also a timeout-retry behaviour to avoid cases where the navigation might not find a path due to a transient obstacle. This sees if the vehicle has made greater than 0.75m progress in the last 10 seconds and if it has not it resends the current goal.