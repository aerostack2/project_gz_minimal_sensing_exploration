#!/bin/bash

usage() {
    echo "  options:"
    echo "      -m: use multi ranger sensor"
    echo "      -i: choose world instance (default: '')"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "mi:rt" opt; do
  case ${opt} in
    m )
      modifier="_multi_ranger"
      ;;
    i )
      instance="${OPTARG}"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[mrt]$ ]]; then
        echo "Option -$OPTARG requires an argument" >&2
        usage
        exit 1
      fi
      ;;
  esac
done

source utils/tools.bash

# Shift optional args
shift $((OPTIND -1))

## OVERIDE MODEL PATH TO GET FIRST MODELS FROM PROJECT (planar_lidar)
export IGN_GAZEBO_RESOURCE_PATH=$PWD/assets/worlds:$PWD/assets/models:$IGN_GAZEBO_RESOURCE_PATH

## DEFAULTS
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}
instance=${instance:=""}

estimator_plugin="ground_truth"
simulation_config="assets/worlds/world_two${modifier}${instance}.json"
rviz_config="sim_config/two${modifier}.rviz"

tmuxinator start -p tmuxinator/session.yml \
      drone_namespace="drone0" \
      simulation_config=${simulation_config} \
      rviz_config=${rviz_config} &
wait

tmuxinator start -p tmuxinator/session_simless.yml \
      drone_namespace="drone1" \
      simulation_config=${simulation_config} &
wait

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -p tmuxinator/rosbag.yml \
      drone_namespace="drone0:drone1" &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -p tmuxinator/keyboard_teleop.yml \
      simulation=true \
      drone_namespace="drone0:drone1" &
  wait
fi

# Attach to tmux session ${drone_ns[@]}, window 0
tmux attach-session -t drone0:path_planning