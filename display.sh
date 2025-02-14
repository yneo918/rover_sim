gnome-terminal -- bash -c "source install/setup.bash; ros2 launch display.launch.py"
gnome-terminal -- bash -c "source install/setup.bash; ros2 launch teleop_core teleop_node.launch.py"
gnome-terminal -- bash -c "source install/setup.bash; ros2 run fake_rover_state_controller teleop2js"
# gnome-terminal -- bash -c "source install/setup.bash; ros2 run virtual_joy virtual_joy"
