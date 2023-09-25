#!/bin/bash

usage() {
    echo "  options:"
    echo "      -m: use multi ranger sensor"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
    echo "      -n: drone namespace, default is drone0"
}

# Arg parser
while getopts "mrtn" opt; do
  case ${opt} in
    m )
      modifier="_multi_ranger"
      ;;
    r )
      record_rosbag="true"
      ;;
    t )
      launch_keyboard_teleop="true"
      ;;
    n )
      drone_namespace="${OPTARG}"
      ;;
    \? )
      echo "Invalid option: -$OPTARG" >&2
      usage
      exit 1
      ;;
    : )
      if [[ ! $OPTARG =~ ^[swrt]$ ]]; then
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
drone_namespace=${drone_namespace:="drone0"}

estimator_plugin="ground_truth"
simulation_config="sim_config/world${modifier}.json"
rviz_config="sim_config/rviz${modifier}.rviz"

tmuxinator start -n ${drone_namespace} -p tmuxinator/session.yml \
      drone_namespace=${drone_namespace} \
      simulation_config=${simulation_config} \
      rviz_config=${rviz_config} &
wait

if [[ ${record_rosbag} == "true" ]]; then
  tmuxinator start -n rosbag -p tmuxinator/rosbag.yml
      drone_namespace=${drone_namespace} &
  wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
  tmuxinator start -n keyboard_teleop -p tmuxinator/keyboard_teleop.yml \
      simulation=true \
      drone_namespace=${drone_namespace} &
  wait
fi

# Attach to tmux session ${drone_ns[@]}, window 0
tmux attach-session -t ${drone_namespace}:path_planning