# Babys-First-3D-Printer

## 1. Start
    rostopic pub -1 /startornot std_msgs/Bool true
    
## 2. Difference

big

    eurobot2023_main
     
small

    eurobot2023_main_small
    
## 3. Bugs and Targets

#### 1. Safest and looking for shortest distance will sometimes plan a strange looking route when some cakes are taken away while robots are moving.
#### 2. first_cherry.py has been updated, but there are still some problems that need to be fixed.
#### 3. Robots will sometimes keep changing planned routes in a strange way due to the algorithm, but if I fix the robot in specific points, it won't happen.
#### 4. Robots will still plan to the same cherryside sometimes, the algorithm still needs to be imporved.
#### 5. The STEAL state hasn't been written.
#### 6. 

## 4. Navigation

https://well-season-c5b.notion.site/a0708dcd585249ecae95b1ce3764befc

navigation_main hasn't been uploaded!!!

## 5. map in rviz
    
#### add in sim~.launch
    <node name="publish_image" pkg="rviz_display" type="publish_image" output="screen"/>
    <node pkg="tf" type="static_transform_publisher" name="quad_broadcaster" args="1.5 1 -0.01 0 0 -1 1 robot1/map quad 30" />

#### add in .rviz and change Static map's alpha to 0
    - Class: rviz_textured_quads/MeshDisplayCustom
      Enabled: true
      Image Topic: /robot1/Eurobot_Image
      Meters per pixel: 0.001465
      Name: MeshDisplayCustom
      Quad Frame: quad
      Value: true

## 6. Drive car

    cd Eurobot_2023/src/Eurobot2023_main_test/src
    python3 haha.py
    
#### (1) mode : p/d (path or dock)
#### (2) robot : 1/2(our robot) 3/4(rival)
#### (3) x : x position in meters
#### (4) y : y position in meters
#### (5) ang : robot's angle in degrees

## 7. Run Car
    
    nano ~/.bashrc

change the last 2 lines to :

    export ROS_MASTER_URI=http://192.168.50.51:11311
    export ROS_IP=192.168.50.51
    
##### (1) 9 for robot1
##### (2) 174 for robot2
##### (3) 210 for rival1
##### (4) 51 for DELL

    source ~/.bashrc
    
open new terminal

1. Port照標籤插 適時把HUB重插

2. roslaunch navigation_run hub.launch --> 機構2個ST

3. roslaunch navigation_run run_robot2.launch --> Lidar, 底盤ST --> 成功顯示黃色beacon distance / geometry error(需開rviz使用2D Pose Estimate調整到正確初始位置，可看牆壁方向/rviz_sim.rviz則要「第一個」2D Pose Estimate)

4. (本機)roslaunch navigation_run open_rviz.launch --> launch內rviz檔案改成rviz2.rviz (或rviz_sim.rviz，但要左側逐個調topic)

## 8. Rosserial

https://hackmd.io/@925/S1lQD_ljq/%2Fs%2Fryg2eQv4Q1Gplup81jfMRw%3Fboth%233-1-ROS-%25E7%25AB%25AF%25E8%25A8%25AD%25E5%25AE%259A

    dmesg | grep tty
    sudo chmod 777 /dev/ttyACM0
    rosrun rosserial_python serial_node.py _baud:=115200 _port:=/dev/ttyACM0
    
