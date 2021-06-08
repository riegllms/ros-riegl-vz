# ROS API

## 1. Nodes

### 1.1 riegl_vz

#### 1.1.1 Parameters

**~hostname** (string, default: "") : 

The scanners hostname or IP address. 

**~ssh_user** (string, default: "user") : 

The linux user name for SSH login on the scanner.

**~ssh_password** (string, default: "user") : 

The linux user password for SSH login on the scanner.

**~scan_pattern** (double[6], default: {0.0,360.0,0.1,0.0,180.0,0.1}) : 

The field of view for the laser scan in angular degree.<br>
[0]: frame_start<br>
[1]: frame_stop<br>
[2]: frame_delta<br>
[3]: line_start<br>
[4]: line_stop<br>
[5]: line_delta

**~meas_prog** (integer, default: 0) :

The laser scanners measurement program, defining the laser pulse repetition rate (PRR).

**~stor_media** (integer, default: 2) :

Configures storage media for measurement files in the scanner (1: auto, 2: internal).

**~project_name** (string, default: "") :

The scanner project name.

**~pointcloud_stream** (integer, default: 0) :

Select the laser scanners output stream for pointcloud data (0: standard measurement stream, 1: reduced monitor stream).

**~pointcloud_msm** (integer[2], default: {10,10}) :

The monitor stream multiplier used for measurement data reduction of the scanners monitor data stream.

**~pointcloud_coordinate_system** (string, default: "SOCS") :

Select the coordinate system for the pointcloud data.<br>
"SOCS" : Scanner own coordinate system<br>
"PCRS" : Project coordinate system

**~registration_max_iterations** (integer, default: 1) :

The number of iterations for scan registration.

**~registration_max_iteration_distance** (double, default: 10.0) :

The maximum distance between the last scan position of the current iteration and the first scan position of the next iteration in meter.


#### 1.1.2 Published Topics

**pointcloud** (sensor_msgs/PointCloud2) : 

Point cloud with scan data from the laser scanner.

**pose** (geometry_msgs/Pose) : 

Position and orientation of the scanner origin in project coordinate system (PCRS).

**status** (???) : 

Riegl VZ scanner status, provided once per second.


#### 1.1.3 Services

**set_project** (std_srvs/Trigger) :

Create a new or load an existing project on the scanner with name from parameter ~project_name, If empty the default name is derived from actual date and time.

Returns:
success = True -> message: Project Name
success = False -> message: Error Message

**scan** (std_srvs/Trigger) : 

Start laser scan and wait until scan is finished. Provide scan data on 'pointcloud' topic.

Returns:<br>
success = True -> message: Measurement Identifier<br>
success = False -> message: Error Message<br>

**coarse_registration** (std_srvs/Trigger) : 

Start laser scan coarse registration and wait until scan is finished. Provide estimated position on 'pose' topic.

Returns:<br>
success = True -> message: SUCCESS<br>
success = False -> message: Error Message<br>

**registration** (std_srvs/Trigger) : 

Start laser scan registration and wait until scan is finished. Provide estimated position on 'pose' topic.

Returns:<br>
success = True -> message: SUCCESS<br>
success = False -> message: Error Message<br>

**shutdown** (std_srvs/Trigger) : 

Shutdown the laser scanner.

Returns:<br>
success = True -> message: SUCCESS<br>
success = False -> message: Error Message<br>
