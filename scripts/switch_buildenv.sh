#!/bin/bash

docker run -v $PWD:/media/workdir -w /media/workdir -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix ros2t1
