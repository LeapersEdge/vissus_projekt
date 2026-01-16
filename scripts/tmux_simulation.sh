#!/bin/bash

# If the tmux session doesn't exist
if ! tmux has-session -t mrs_simulation 2>/dev/null; then
  # Create tmux session
  tmux new-session -d -s mrs_simulation -n main
  tmux new-window -t mrs_simulation -n bg
  
  # Split window
  tmux split-window -h -t mrs_simulation:main # main.0 ~ left, main.1 ~ right
  tmux split-window -v -t mrs_simulation:main.1 # main.1 ~ top-right, main.2 ~ bottom-right
fi

# Run the commands to setup the simulation
tmux send-keys -t mrs_simulation:bg "cd ~/ros2_ws/src/mrs_crazyfiles/startup" Enter
tmux send-keys -t mrs_simulation:bg "./start.sh" Enter
tmux send-keys -t mrs_simulation:bg "pgrep -f rqt || rqt &" Enter
tmux send-keys -t mrs_simulation:main.1 "ros2 launch simulation.launch.py" Enter

# Open the tmux session
tmux attach-session -t mrs_simulation

# To-do if possible: rqt should start with a publisher on tuning_params and a publisher on cp4/cmd_vel
