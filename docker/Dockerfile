FROM osrf/ros:galactic-desktop

RUN apt-get update \
	&& apt-get install -y \
	vim \
	pip \
	ros-galactic-diagnostic-updater \
	xauth

RUN touch /root/.Xauthority

ADD src /opt/src
RUN cd /opt/src && python3 -m pip install -r requirements.txt
RUN cd /opt/src/librdb && pip3 install riegl.rdb-2.3.4-cp34.cp35.cp36.cp37.cp38.cp39-none-linux_x86_64.whl
