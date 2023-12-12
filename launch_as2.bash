#!/bin/bash

usage() {
    echo "Usage: $0 [-i <instance>] [-2] [-3] [-r] [-t]"
    echo "  options:"
    echo "      -i: choose world instance (default: '60')"
    echo "      -2: launch two drones"
    echo "      -3: launch three drones"
    echo "      -r: record rosbag"
    echo "      -t: launch keyboard teleoperation"
}

# Arg parser
while getopts "23i:rth" opt; do
    case ${opt} in
        2 )
            n_drones="two"
            ;;
        3 )
            n_drones="three"
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
instance=${instance:="60"}
n_drones=${n_drones:="one"}

estimator_plugin="ground_truth"
simulation_config="assets/worlds/world${instance}_${n_drones}_drone.json"
rviz_config="assets/rviz/${n_drones}_drone.rviz"

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