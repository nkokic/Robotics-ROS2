Pokretanje simulacije + stvaranje rupe:
    ros2 launch ur5_robotiq ur5_robot_sim_moveit.launch.py

Nuliranje senzora:
    ros2 run robot_control sensor_nulling

Stavljanje valjka u rupu:
    ros2 run robot_control put_peg_in_hole

Samo stvaranje rupe: (trebat će ponovno pokretati simulaciju za novu nasumičnu poziciju, stoga nije nužno potrebno)
    ros2 run ur5_robotiq spawn_hole_random