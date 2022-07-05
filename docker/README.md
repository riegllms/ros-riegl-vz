# Running 'riegl-vz' ROS node in a docker container

**Build docker container:**

Git clone or extract source into local working directory to subfolder 'src'.   
Stay in local working directory and build the docker container, using a network proxy configuration:

Galactic Geochelone:

```docker build . -t ros2galactic -f src/docker/Dockerfile.galactic --build-arg http_proxy=http://<ip-address>:<port-number> --build-arg https_proxy=http://<ip-address>:<port-number>```  

Humble Hawksbill:
 
```docker build . -t ros2humble -f src/docker/Dockerfile.humble --build-arg http_proxy=http://<ip-address>:<port-number> --build-arg https_proxy=http://<ip-address>:<port-number>```  

**Run docker container:**

Galactic Geochelone:

```docker run -v $PWD:/media/workdir -w /media/workdir -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix ros2galactic```  

Humble Hawksbill:

```docker run -v $PWD:/media/workdir -w /media/workdir -it --net=host -e DISPLAY=$DISPLAY -e PYTHONWARNINGS='ignore:::setuptools.command.install' -v /tmp/.X11-unix ros2humble```  

Now you can build and execute the ROS node in the docker container.

**Start another bash in an already running container:**

```docker ps```

```
CONTAINER ID   IMAGE           COMMAND                  CREATED          STATUS          PORTS     NAMES
56cbea55fe70   ros2galactic    "/ros_entrypoint.sh …"   15 seconds ago   Up 14 seconds             condescending_merkle
```

```docker exec -it 56cbea55fe70 bash```

In the container setup the ROS environment:

```. install/setup.bash```

**Run rviz in a docker container:**

Get xauth from host:

```xauth list```

```
compaf/unix:  MIT-MAGIC-COOKIE-1  74e2aeecbb5cb3e827559b0bee7e2375
#ffff#636f6d706166#:  MIT-MAGIC-COOKIE-1  74e2aeecbb5cb3e827559b0bee7e2375
```

Add xauth in running docker container:

```xauth add compaf/unix:0  MIT-MAGIC-COOKIE-1  74e2aeecbb5cb3e827559b0bee7e2375```

Execute graphical application in the docker container:

```rviz2```
