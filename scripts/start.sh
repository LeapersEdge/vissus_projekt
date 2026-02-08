#!/bin/bash

# Absolute path to this script
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
cd "$SCRIPTPATH"

SETUP_NAME=$1
[ -z "$SETUP_NAME" ] && SETUP_NAME=mrs_example_setup.sh

# Start tmuxinator session
tmuxinator start -p example.yml setup_name=$SETUP_NAME

