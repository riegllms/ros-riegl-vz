std_msgs/Header header
uint32 voxel_count
uint8[] data
# data is a byte array containing a list of voxels. Each voxel has following data fields:
# float64[3] xyz                : Cartesian point coordinates wrt. application coordinate system (0: X, 1: Y, 2: Z)
# float32[3] pca_axis_min       : The eigenvector that belongs to the smallest eigenvalue (result of PCA, 0: X, 1: Y, 2: Z)
# float32[3] pca_axis_max       : The eigenvector that belongs to the greatest eigenvalue (result of PCA, 0: X, 1: Y, 2: Z)
# float32 reflectance           : Target surface reflectance
# uint32 point_count            : Number of points this point represents (e.g. points combined to voxels or plane patches, 0 = unknown)
# float32[3] pca_extents        : Volume extents along 0: pca_axis_max, 1: pca_axis_min x pca_axis_max, 2: pca_axis_min
# bool voxel_collapsed          : Voxel has been collapsed with neighbor (0 = not collapsed, 1 = collapsed)
# uint8 shape_id                : Estimated shape of point cloud (0 = undefined, 1 = plane, 2 = line, 3 = volume)
# float64[6] covariances        : Elements 00, 11, 22, 10, 21 and 20 (in that order) of point cloud covariance matrix
# uint64 id                     : Point identifier, unique within database (0 = invalid)
