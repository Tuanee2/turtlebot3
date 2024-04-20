USER MANUAL:
    Make sure you have a map.
    exp: demo_map , demo_map.yaml

1) Localization
    Run this cmd.
    ~$ ros2 launch my_launch my_localization.launch.py map:=$HOME/demo_map.yaml

    Open new terminal and run this cmd.
    ~$ ros2 run my_launch init_pos_pub 

    In rviz2 use the "publish point" to select the robot's estimated position. Then, using teleop_key node to move your robot around to to make the estimated coordinates more accurate.

2) Create and send the PATH
    Open new terminal and run this cmd.
    ~$ ros2 run my_launch create_path

    Using Rviz2 to select points and they will be save in the path (maximum 30 points). Press 's' or 'S' to send the PATH to "/plan" topic and terminate "create_path" node.


