#!/bin/bash

# allow any application to connect to the X server
xhost +local:docker

# run docker with x11 forwarding
sudo docker run -it \
    --gpus all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name=apex_mr-container \
    apex_mr-image
