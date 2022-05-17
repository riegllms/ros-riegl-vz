# ROS2 RIEGL VZ Package

This is a ROS2 package for the RIEGL VZ-i series devices.
The package is based on ROS2 [Galactic Geochelone](https://docs.ros.org/en/galactic/index.html) distribution and has been tested on Ubuntu Desktop 20.04 (Focal Fossa).

It provides a ROS interface with services and topics for scan data acquisition and scan position registration with a 3D laser scanner.  
In detail, it provides the following functions:

* Create a project on the laser scanner device

* Configure the the laser scanner, including configuration of the scan pattern and the measurement program as well as the storage device for data logging on the device

* Start a scan data acquisition with
  - optional scan registration
  - optional reflector scan and search
  - optional camera image capture

* Publish the 3D point cloud in ROS

* Publish voxel grid in ROS

* Publish scan position resulting from scan registration, position is in scanner project coordinates or referenced to a eglobal coordinate system if required

* Use external kinematic data from a robot to improve the scan position registration algorithm.

* Publish the device status (memory usage, error stack, ...) and data acquisition progress information

* Publish positional data from the GNSS receiver in the laser scanner

**ROS2 installation and setup:**

Follow ROS2 installation instructions on https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html.

Configure the ROS2 environment according to https://docs.ros.org/en/galactic/Tutorials/Configuring-ROS2-Environment.html.

Create a new workspace (https://docs.ros.org/en/galactic/Tutorials/Workspace/Creating-A-Workspace.html)
and clone repository into subdirectory 'src'.

**Install diagnostics updater package:**

```sudo apt-get install ros-galactic-diagnostic-updater```

**Install tf2 packages:**

```sudo apt-get install ros-galactic-tf2-tools ros-galactic-tf-transformations```

**Install python requirements:**

Switch to 'src' subdirectory and install required python modules:

```python3 -m pip install -r requirements.txt```

**Install python wheel with rdb library:**

Find an appropriate python wheel for the rdb library in the 'librdb/' subfolder of the repository  
or send a request to RIEGL support: support@riegl.com.  
The wheel includes a shared linux library must be appropriate for the target processor architecture.  
Install the wheel, e.g. for x86_64:

```pip3 install riegl.rdb-2.3.4-cp34.cp35.cp36.cp37.cp38.cp39-none-linux_x86_64.whl```

**Build package:**

Switch to workspace root directory.

```colcon build```

**Start 'riegl_vz' node:**

Open a second terminal. Execute '. install/setup.bash'.

Find .yaml files for configuration of node parameters in package install directory at: **install/riegl_vz/share/riegl_vz/config/**.

Copy **params_default.yaml** to **params.yaml** and edit parameter settings.

Launch 'riegl_vz' node with parameter settings from params.yaml:

```ros2 launch riegl_vz std_launch.py```

**Trigger a scan data acquisition:**

Open another terminal. Execute '. install/setup.bash'.

Execute the scan trigger service:

```ros2 service call /scan std_srvs/srv/Trigger```

**Visualize scan data point cloud with rviz:**

Start 'rviz' tool:

```rviz2```

Set 'fixed-frame' in 'Global Options' to 'riegl_vz_socs'. Activate 'PointCloud2' plugin and bind it to riegl_vz/pointcloud topic:

![rviz2](riegl_vz/img/rviz2.png)

**Monitor 'riegl_vz' node with foxglove-studio:**

Install foxglove-studio from https://foxglove.dev.

Start 'foxglove-studio':

```foxglove-studio```

Import layout from foxglove/diagnostics.json..

and open a data source, e.g. open live connection.

![rqt](riegl_vz/img/foxglove.png)
