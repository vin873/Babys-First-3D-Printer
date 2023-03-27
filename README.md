# Babys-First-3D-Printer

## 1.Big

### catkin_make
    cd Eurobot_2023/
    source devel/setup.bash
    catkin_make
    
### roslaunch
    source devel/setup.bash
    roslaunch Eurobot2023_main_test main.launch
    
## 2.Small

### catkin_make
    cd Eurobot_2023_small/
    source devel/setup.bash
    catkin_make
    
### roslaunch
    source devel/setup.bash
    roslaunch Eurobot2023_main_small_test main.launch

## 3.Start
    rostopic pub -1 /startornot std_msgs/Bool true
    
## 4.Difference

big

    Eurobot2023_main_test
    mainClass.poseStamped_set(0, home[0], 1.125, 1.980, 0, 1);
    mainClass.poseStamped_set(0, home[1], 1.125, 0.020, 0, 1);
     
small

    Eurobot2023_main_small_test
    mainClass.poseStamped_set(0, home[0], 1.125, 1.680, 0, 1);
    mainClass.poseStamped_set(0, home[1], 1.125, 0.320, 0, 1);
    
## 5. Bug that hasn't been fixed

#### 1. safest but looking for shortest distance
#### 2. sometimes cherry won't find the closest side
#### 3. sometimes cherry again will do the cherry side that has been done (not important)
