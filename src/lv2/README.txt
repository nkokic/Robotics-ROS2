Za pokretanje simulacije, stvaranja robota u okruženju te pokretanje navigacijskog stoga
    ros2 launch robot_nav2_bringup nav2_mob_rob.launch.xml
NAPOMENA! Ako snimate svoj bag, izbrišite postojeći "robot_cilj" direktorij jer neće prepisati postojeći 

Nakon što se kreira ros bag, vizualizacija mape i kretanja robota je pomoću naredbe
    ros2 run robot_nav2_navigation bag_visualizer ./robot_cilj
Ovdje je "./robot_cilj" putanja do ros bag direktorija.