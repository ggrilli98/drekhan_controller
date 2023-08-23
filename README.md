# drehkran_controller

# Drehkran Controller
ROS 2 Package for the drehkran show.

### 1. SSH into drehkran and restart the services

    sudo systemctl restart ec_interface.service ec_control.service ec_loadcell.service urg_node.service

### 2. Sending commands from remote PC
    
    source /opt/ros/foxy/setup.bash
    cd ~/ros2_ws
    source install/setup.bash
    ros2 run drehkran_controller drehkran_routine.py 
    
### 3. Sending commands from joystick (remote PC)

    ros2 launch joy_drehkran joy_drehkran_launch.py 
