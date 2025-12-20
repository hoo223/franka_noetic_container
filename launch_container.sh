sudo docker run -it --net=host --privileged \
    --cap-add=sys_nice \
    --ulimit rtprio=99 \
    --ulimit memlock=-1 \
    -e DISPLAY=unix$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /usr/local/share/ca-certificates:/usr/local/share/ca-certificates \
    -v /usr/share/ca-certificates:/usr/share/ca-certificates \
    -v `pwd`/share:/root/share \
    -w /root/share \
    --name franka_noetic \
    osrf/ros:noetic-desktop-full
    bash