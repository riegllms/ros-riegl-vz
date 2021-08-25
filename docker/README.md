# Run ROS node:

**build ros Docker:**
```docker build -t ros2t1 .```

**create a build mount:**
```
mkdir workdir
cd workdir
git clone git@gitlab.riegl-gmbh:riegl/firmware/ros-riegl-vz.git
cd ..
```

**run ros Docker:**
```docker run -v "$PWD"/workdir:/media/workdir -w /media/workdir -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix ros2t1```

**Install python requirements:**
```
cd ros-riegl-vz
python3 -m pip install -r requirements.txt
```

**Install librdb python wheel:**
```
cd librdb
pip3 install riegl.rdb-2.3.4-cp34.cp35.cp36.cp37.cp38.cp39-none-linux_x86_64.whl
cd ..
```

**Build package:**
```colcon build```

**Start 'riegl_vz' node:**
```. install/local_setup.sh```

**configure node parameters**
```
cp install/riegl_vz/share/riegl_vz/config/params_default.yaml install/riegl_vz/share/riegl_vz/config/params.yaml
vi install/riegl_vz/share/riegl_vz/config/params.yaml
```

**Launch 'riegl_vz' node with parameter settings from params.yaml:**
```ros2 launch riegl_vz std_launch.py```

**open 2nd terminal:**
```
docker ps
docker exec -it aca9ba982d7a bash
```

```. /opt/ros/galactic/local_setup.sh```

**Execute the scan trigger service:**
```ros2 service call /scan std_srvs/srv/Trigger```


# Visualize scan data point cloud with rviz:

**get xauth form host <cookie>**
```xauth list```

**run docker**
docker run -v "$PWD"/workdir:/media/workdir -w /media/workdir -it --net=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix ros2t1

xauth add <cookie>
```
xauth add zr-comp/unix:0  MIT-MAGIC-COOKIE-1  db11fa96b11f98b443129ab7e1cbxxx
xauth list
```

**run programm** 
```rviz2```

