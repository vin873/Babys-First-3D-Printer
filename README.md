# Babys-First-3D-Printer

## 1.Big

### catkin_make
    roscore
    cd Eurobot_2023/
    source devel/setup.bash
    catkin_make
    
### roslaunch
    source devel/setup.bash
    roslaunch Eurobot2023_main_test main.launch

### mission finish
    cd src/Eurobot2023_main_test/src/
    python3 test_pub.py 
    
## 2.Small

### catkin_make
    roscore
    cd Eurobot_2023/
    source devel/setup.bash
    catkin_make
    
### roslaunch
    source devel/setup.bash
    roslaunch Eurobot2023_main_test main.launch

### mission finish
    cd src/Eurobot2023_main_test/src/
    python3 test_pub.py 

## 3.Start
    rostopic pub -1 /startornot std_msgs/Bool true
