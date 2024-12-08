#!/bin/bash

ROS_WORKSPACE_PATH="$1"

docker run -it \
--env DISPLAY \
--volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
--volume $ROS_WORKSPACE_PATH:/lvi_sam_ws \
--name lvi_sam_container \
--detach lvi_sam