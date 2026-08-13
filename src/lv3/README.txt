Pokretanje simulacije i dodavanja ormarića:
    ros2 launch ur5_robotiq_sim ur5_robot_sim_moveit.launch.py

Mijenjanje pozicije ormarića se odvija u dvije datoteke:
1. ur5_robot_sim_moveit.launch.py -> spawn_cabinet -> arguments
2. robot_control.py -> cabinetPose

Pokretanje otvaranja ormarića:
* ros2 run robot_control robot_control
ili
* otvoriti robot_control.py u visual studio code te aktivirati "run python file"

Ponekad robot ne izvršava sve putanje stoga je nekada potrebno ponovno pokrenuti robot_control!
