# This message holds the description of one point entry in the
# VoxelGrid message format.
uint8 UINT8   = 1
uint8 UINT32  = 2
uint8 UINT64  = 3
uint8 FLOAT32 = 4
uint8 FLOAT64 = 5

# Common VoxelField names are x, y, z, intensity, rgb, rgba
string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field
