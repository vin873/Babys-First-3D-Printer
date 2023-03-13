# Babys-First-3D-Printer

### catkin_make
    roscore
    cd Eurobot_2023/
    source devel/setup.bash
    catkin_make
    
### roslaunch

    source devel/setup.bash
    roslaunch Eurobot2023_main_test main.launch

### start

    rostopic pub -1 /startornot std_msgs/Bool true
    
