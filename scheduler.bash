#!/bin/bash

## OVERIDE MODEL PATH TO GET FIRST MODELS FROM PROJECT (planar_lidar)
export GZ_SIM_RESOURCE_PATH=$PWD/assets/worlds:$PWD/assets/models:$GZ_SIM_RESOURCE_PATH
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export IGN_IP=127.0.0.1

do_explorations() {
    # $1: number of iterations
    # $2: simulation config file
    # $3: rviz config file
    n=$1
    simulation_config=$2
    rviz_config=$3

    # Get drone namespaces from swarm config file
    drones=$(python utils/get_drones.py ${simulation_config}) 

    for ((i = 0 ; i < $n ; i++)); do
        printf "\n --------- ITERATION $i --------- \n"
        echo "$(date '+%Y%m%d_%H%M%S') ${simulation_config} IT $i" | tee -a EXPERIMENTS.md

        tmuxinator start -p tmuxinator/gazebo.yml \
            simulation_config=${simulation_config} &
        sleep 30
        
        tmuxinator start -p tmuxinator/aerostack2.yml \
            drones=${drones} \
            simulation_config=${simulation_config} \
            rviz_config=${rviz_config} &

        sleep 10
        python explore.py
        ./kill.bash
        pskill -y gz
    done
}

simulation_config="assets/worlds/low3.json"
rviz_config="assets/rviz/three_drone.rviz"

do_explorations 5 ${simulation_config} ${rviz_config}