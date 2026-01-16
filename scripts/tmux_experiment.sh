#!/bin/bash

# If the tmux session doesn't exist
if ! tmux has-session -t mrs_experiment 2>/dev/null; then
  # Create tmux session
  tmux new-session -d -s mrs_experiment -n main
  tmux new-window -t mrs_experiment -n bg
  
  # Split window
  tmux split-window -h -t mrs_experiment:main # main.0 ~ left, main.1 ~ right
  tmux split-window -v -t mrs_experiment:main.1 # main.1 ~ top-right, main.2 ~ bottom-right
fi

# Run the commands to setup the experiment
tmux send-keys -t mrs_experiment:bg "cd ~/ros2_ws/src/mrs_crazyfiles_exp/startup" Enter
tmux send-keys -t mrs_experiment:bg "./start.sh" Enter
tmux send-keys -t mrs_experiment:bg "pgrep -f rqt || rqt &" Enter
tmux send-keys -t mrs_experiment:main.1 "ros2 launch simulation.launch.py" Enter

# Open the tmux session
tmux attach-session -t mrs_experiment

# To-do if possible: rqt should start with a publisher on tuning_params and a publisher on cp4/cmd_vel
