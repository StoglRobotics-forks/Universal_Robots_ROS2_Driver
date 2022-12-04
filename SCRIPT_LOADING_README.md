## Before starting

- Setup [custom_script1.urscript](./ur_robot_driver/resources/custom_script_1.urscript):

  - Setup up address of computer running driver - `COMPUTER_DRIVER_IP` variable
    (if using `ursim` in docker then: "192.168.56.1")


## Starting

1. Start robot or [`ursim`](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html#usage-with-official-ur-simulator)

2. Start driver: ros2 launch ur_bringup ur_control.launch.py ur_type:=ur16e robot_ip:=<ROBOT_IP> headless_mode:=true launch_rviz:=false

   - For `ursim`: ros2 launch ur_bringup ur_control.launch.py ur_type:=ur16e robot_ip:=192.168.56.101 headless_mode:=true launch_rviz:=false

3. Start node for shared variables:
   ```
   ros2 launch ur_bringup monitoring_node.launch.py robot_ip:=<ROBOT_IP>
   ```

4. Control robot using MoveIt2 and ros2_control

5. Switch script using - robots starts to exacute cyclic movements
   ```
   ros2 service call /io_and_status_controller/switch_script std_srvs/srv/Trigger {}
   ```

6. Setup movement distance using montoring node:
   ```
   ros2 topic pub /manitoring_node/var0 std_msgs/msg/Float64 "{data: 0.15}"
   ```

7. Stop/Terminate script execution:
   ```
   ros2 service call /shared_variables_node/stop_main_loop std_srvs/srv/Trigger {}
   ```

8. Switch again `ros2_control` script to use MoveIt2/ros2_control:
   ```
   ros2 service call /io_and_status_controller/resend_robot_program std_srvs/srv/Trigger {}
   ```
