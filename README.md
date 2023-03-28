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

#### 1. Safest and looking for shortest distance will sometimes plan a strange looking route when some cakes are taken away while robots are moving.
#### 2. Sometimes cherry won't find the closest side.
#### 3. Sometimes cherry again will go to the cherry side that has been done. (not important)
#### 4. Cherries will plan the same side as the enemy.

## 6. Navigation

https://well-season-c5b.notion.site/a0708dcd585249ecae95b1ce3764befc
