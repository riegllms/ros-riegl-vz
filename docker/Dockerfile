FROM osrf/ros:galactic-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y \
    && apt-get install -y \
    vim \
    pip \
    ros-galactic-tf2-tools \
    ros-galactic-tf-transformations \
    ros-galactic-diagnostic-updater \
    libnotify4 libxtst6 xdg-utils \
    wget \
    xauth

RUN touch /root/.Xauthority

ADD src /opt/src
RUN cd /opt/src && python3 -m pip install -r requirements.txt
RUN cd /opt/src/librdb && pip3 install riegl.rdb-2.3.7-cp34.cp35.cp36.cp37.cp38.cp39-none-linux_x86_64.whl

ENV FOXGLOVE_STUDIO_VERSION=0.29.0
RUN wget -P /tmp/ https://github.com/foxglove/studio/releases/download/v0.29.0/foxglove-studio-${FOXGLOVE_STUDIO_VERSION}-linux-amd64.deb
RUN dpkg -i /tmp/foxglove-studio-${FOXGLOVE_STUDIO_VERSION}-linux-amd64.deb
