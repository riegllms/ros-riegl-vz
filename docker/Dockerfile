FROM osrf/ros:galactic-desktop

ENV http_proxy "http://192.168.0.9:3128"
ENV https_proxy "http://192.168.0.9:3128"

RUN apt-get update \
	&& apt-get install -y \
	vim \
	pip \
	ros-galactic-diagnostic-updater \
	xauth
