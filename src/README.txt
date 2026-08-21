Pokretanje simulacije i navigacije:
    ros2 launch amr_lv4 rosbot_nav.launch.py

Prikupljanje patrolnih točaka i položaja markera:
    ros2 launch amr_lv4 collect_points.launch.py \
        max_points:=5 \
        patrol_output_file:=./src/amr_lv4/patrol_points.yaml \
        marker_output_file:=./src/amr_lv4/config/aruco_markers.yaml

    U RVizu koristi "Publish Point" za patrolne točke i "2D Goal Pose" za
    markere. Dodaj MarkerArray prikaz s temom /collected_points za prikaz
    plavih patrolnih točaka i narančastih orijentacija markera.

Stvaranje ArUco markera i pokretanje patrole:
    ros2 launch amr_lv4 spawn_and_patrol.launch.py \
        marker_file:=./src/amr_lv4/config/aruco_markers.yaml \
        patrol_file:=./src/amr_lv4/patrol_points.yaml \
        wait_time:=5.0 \
        stop_distance:=1.0


Rezerve
_________________________________________________________________________________________________

Prikupljanje točaka:
    ros2 launch amr_lv4 gather_points.launch.py max_points:=5 output_file:=./src/amr_lv4/patrol_points.yaml

Obilazak točaka:
    ros2 launch amr_lv4 patrol_navigator.launch.py input_file:=./src/amr_lv4/patrol_points.yaml

Prikupljanje položaja markera:
    ros2 run amr_lv4 collect_marker_poses

Stvaranje markera:
    ros2 launch amr_lv4 spawn_aruco_markers.launch.py

Stvaranje markera iz izravno ažurirane izvorne konfiguracije (bez rebuilda):
    ros2 launch amr_lv4 spawn_aruco_markers.launch.py \
        marker_file:=./src/amr_lv4/config/aruco_markers.yaml

Obilazak točaka i traženje aruco markera:
    ros2 run amr_lv4 aruco_patrol --ros-args \
        -p patrol_file:=./src/amr_lv4/patrol_points.yaml \
        -p wait_time:=5.0 \
        -p stop_distance:=1.0
