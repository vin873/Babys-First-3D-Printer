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
    
### arrive
    rostopic pub -1 /finishornot std_msgs/Bool true

### mission finish
    cd src/Eurobot2023_main_test/src/
    python3 test_pub.py 

### echo mission
    rostopic echo /mission

### echo chassis
    rostopiv echo /wherego
