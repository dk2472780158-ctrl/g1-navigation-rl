ros2 run g1_robot_control g1_loco_client_example --get_fsm_id

ros2 run g1_robot_control g1_loco_client_example --set_fsm_id=

ros2 run g1_robot_control g1_loco_client_example --set_velocity="0.3 0.0 0.0 2.0"

ros2 run g1_robot_control g1_cmd_vel_bridge

ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 launch g1_fastlio_bridge g1_fastlio_mapping.launch.py

ros2 service call /map_save std_srvs/srv/Trigger "{}"