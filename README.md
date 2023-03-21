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
    cd Eurobot_2023_small/
    source devel/setup.bash
    catkin_make
    
### roslaunch
    source devel/setup.bash
    roslaunch Eurobot2023_main_small_test main.launch

### mission finish
    cd src/Eurobot2023_main_small_test/src/
    python3 test_pub.py 

## 3.Start
    rostopic pub -1 /startornot std_msgs/Bool true
    
## 4.Difference

big

    mainClass.poseStamped_set(home, 1.125, 0.020, 0, 1);
     
small

    mainClass.poseStamped_set(home, 1.125, 0.420, 0, 1);
