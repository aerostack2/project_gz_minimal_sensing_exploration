#!/bin/bash

usage() {
    echo "Usage: $0 [-r] [-t]"
    echo "  options:"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "rth" opt; do
    case ${opt} in
        r )
            record_rosbag="true"
            ;;
        t )
            launch_keyboard_teleop="true"
            ;;
        h )
            usage
            exit 0
            ;;
        \? )
            echo "Invalid option: -$OPTARG" >&2
            usage
            exit 1
            ;;
        : )
            if [[ ! $OPTARG =~ ^[rt]$ ]]; then
                echo "Option -$OPTARG requires an argument" >&2
                usage
                exit 1
            fi
            ;;
    esac
done

# Shift optional args
shift $((OPTIND -1))

## OVERIDE MODEL PATH TO GET FIRST MODELS FROM PROJECT (planar_lidar)
export IGN_GAZEBO_RESOURCE_PATH=$PWD/assets/worlds:$PWD/assets/models:$IGN_GAZEBO_RESOURCE_PATH

## DEFAULTS
record_rosbag=${record_rosbag:="false"}
launch_keyboard_teleop=${launch_keyboard_teleop:="false"}

# CHOOSE SIMULATION CONFIG FILE
echo "Choose simulation config file to open:"
cat -n <(ls -1 assets/worlds/*.json) # list json files
simulation_config=$(python utils/pick_sim_config.py assets/worlds | tail -n 1)
if [[ ${simulation_config} == "Invalid" ]]; then
    echo "Invalid option" >&2
    exit 1
fi

estimator_plugin="ground_truth"
rviz_config="assets/rviz/three_drone.rviz"

# Get drone namespaces from swarm config file
drones=$(python utils/get_drones.py ${simulation_config})

tmuxinator start -p tmuxinator/aerostack2.yml \
    drones=${drones} \
    simulation_config=${simulation_config} \
    rviz_config=${rviz_config} &
wait

if [[ ${record_rosbag} == "true" ]]; then
    tmuxinator start -p tmuxinator/rosbag.yml \
        drones=${drones} &
    wait
fi

if [[ ${launch_keyboard_teleop} == "true" ]]; then
    # TODO: Keyboard Teleop uses ',' as separator for drone namespaces
    drones_sep=$(python utils/get_drones.py ${simulation_config} --sep ",")
    tmuxinator start -p tmuxinator/keyboard_teleop.yml \
        simulation=true \
        drones=${drones_sep} &
    wait
fi

# Attach to tmux session aerostack2, window 0
tmux attach-session -t aerostack2:alphanumeric_viewer