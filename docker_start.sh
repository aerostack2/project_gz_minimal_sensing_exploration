
docker run -it --gpus=all --device /dev/dri/ \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /tmp/.docker.xauth:/tmp/.docker.xauth:rw \
    -e XAUTHORITY=/tmp/.docker.xauth \
    -v $(pwd):/home/cvar/path_planning:rw \
    as2:rolling