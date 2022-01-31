FROM osrf/ros:galactic-desktop

RUN apt-get update \
    && apt-get install -y \
    vim \
    pip \
    ros-galactic-tf2-tools \
    ros-galactic-tf-transformations \
    ros-galactic-diagnostic-updater \
    ros-galactic-plotjuggler-ros \
    xauth

RUN touch /root/.Xauthority
RUN echo 'ros2 run plotjuggler plotjuggler' > /opt/ros/galactic/bin/plotjuggler && chmod a+x /opt/ros/galactic/bin/plotjuggler

ADD src /opt/src
RUN cd /opt/src && python3 -m pip install -r requirements.txt
RUN cd /opt/src/librdb && pip3 install riegl.rdb-2.3.7-cp34.cp35.cp36.cp37.cp38.cp39-none-linux_x86_64.whl
