IMPORTANT NOTE: PLEASE OPERATE THE ROBOT SAFELY AND ALWAYS KEEP A HAND ON THE EMERGENCY BUTTON

Robot: ping 192.168.1.10

FT: ping 192.168.1.1


-> Real : rosed tum_ics_ur10_controller_tutorial configUR10.ini

1 roslaunch tum_ics_ur_robot_manager robot_script_manager_ur10.launch

2 roslaunch tum_ics_lacquey_gripper_driver lacquey_gripper_driver_ur10.launch

3 roslaunch tum_ics_schunk_netbox sensor_publisher.launch

4 roslaunch tum_ics_ur10_bringup bringUR10-FT-lacquey.launch

5 roslaunch tum_ics_ur10_controller_tutorial adaptive_control.launch


Call the service to turn on admittance control when the robot is in hold position(has lifted the crate).

-> rosservice call /toggle_admittance_controller "data: true"















