Pokretanje simulacije i navigacije:
    ros2 launch amr_lv4 rosbot_nav.launch.py

Prikupljanje točaka:
    ros2 launch amr_lv4 gather_points.launch.py max_points:=5 output_file:=./src/amr_lv4/patrol_points.yaml

Obilazak točaka:
    ros2 launch amr_lv4 patrol_navigator.launch.py input_file:=./src/amr_lv4/patrol_points.yaml

Prikupljanje položaja markera:
    ros2 run amr_lv4 collect_marker_poses

Stvaranje markera:
    ros2 launch amr_lv4 spawn_aruco_markers.launch.py

Obilazak točaka i traženje aruco markera:
    ros2 run amr_lv4 aruco_patrol --ros-args \
        -p patrol_file:=./src/amr_lv4/patrol_points.yaml \
        -p wait_time:=5.0 \
        -p stop_distance:=1.0
