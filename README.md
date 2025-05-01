# Running the Sim

1. Build the code in the main *ENAE450_tb_ws* folder

`colcon build`

2. Source the *install* folder (do this in 2 different terminals as you'll be using 2!)

`source install/setup.bash`

3. Export the turtlebot model

`export TURTLEBOT3_MODEL=waffle_pi`

4. Launch the world

`ros2 launch turtlebot3_gazebo maze_X.launch.py` *(replace X with 0,1, or 2)*

5. Run controller code in the second terminal

`ros2 run turtlebot3_controller executable` *(replace executable with one in the setup.py file)*
