gnome-terminal -- bash -c "source install/setup.bash; ros2 launch rover_description pioneer_display.launch.py; exec bash"
gnome-terminal -- bash -c "source install/setup.bash; ros2 run fake_rover_state_controller teleop2js; exec bash"
gnome-terminal -- bash -c "source install/setup.bash; ros2 run virtual_joy virtual_joy; exec bash"
