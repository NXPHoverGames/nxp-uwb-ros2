python3 gz_twr_point.py &
python3 ros_gz_twr_data_generator.py &
python3 twr_point_publisher.py &
ros2 topic pub --once /joy sensor_msgs/msg/Joy "{buttons: [0, 0,0, 0, 0, 0, 0, 1]}" &
ros2 run ros_gz_bridge parameter_bridge /uwb_twr_point@geometry_msgs/msg/Point@gz.msgs.Vector3d &
python3 demo_control.py