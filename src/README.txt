Pokretanje simulacije robota:
    ros2 launch robot_tb4_bringup rob_manip.launch.xml

Zbog toga što se uvijek instancira na sredinu mape, dana je početna poza te nije potrebno stavljati 2D pose estimate.
Ponekad ne radi iz prve te je potreban restart.


Postavljanje () patrolnih točaka s "Publish Point":
    ros2 launch amr_lv3 gather_patrol_points.launch.py max_points:=4 output_file:=patrol_points.yaml

Patrola i detekcija zelenog objekta: 
    ros2 launch amr_lv3 patrol_with_object_detection.launch.py input_file:=./src/amr_lv3/patrol_points.yaml safe_distance:=1.0 wait_time:=3.0
