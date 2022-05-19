std_msgs/Header header
uint32 seq                      # scan position number within a project, starting with 1, 0 is the current scan position
geometry_msgs/Point position    # position of the reflector
geometry_msgs/Vector3 normal    # normal vector to the reflector surface
float32 diameter                # calculated reflector diameter in meter
string model_name               # reflector model name
string name                     # reflector name
uint32 point_count              # number of measurements
float32 reflectance             # reflectance of the reflector in dB
