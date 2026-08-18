Za pokretanje simulacije, stvaranja robota u okruženju te pokretanje navigacijskog stoga
    ros2 launch robot_nav2_bringup nav2_mob_rob.launch.xml
NAPOMENA! Svako snimanje stvara novi rosbag2 direktorij s vremenskom oznakom.

Nakon što se kreira ros bag, vizualizacija mape i kretanja robota je pomoću naredbe
    ros2 run robot_nav2_navigation bag_visualizer ./rosbag2_2026_08_18-16_26_15 --map /home/student/Robotics-ROS2/src/lv2/robot_nav2_bringup/maps/depot.yaml

Putanja do bag direktorija: ./rosbag2_2026_08_18-16_26_15
Putanja do direktorija mape: /home/student/Robotics-ROS2/src/lv2/robot_nav2_bringup/maps/depot.yaml