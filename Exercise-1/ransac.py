# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


# Voxel Grid filter
##Create a VoxelGrid filter object for our input point cloud 
vox = cloud.make_voxel_grid_filter()
##Choose a voxel (or leaf) size 
LEAF_SIZE = .01
##Set the voxel (or leaf) size 
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
##Call the filter function to obtain the resultant downsampled point cloud 
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)

# PassThrough filter
##Create a PassThrough filter object
passthrough = cloud_filtered.make_passthrough_filter()
axis_min = 1
axis_max = 1.1
passthrough.set_filter_limits(axis_min, axis_max)
##Finally, use the filter function to obtain the resultant point cloud 
cloud_filtered = passthrough.filter()
filename = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename)

# RANSAC plane segmentation
##Create the segmentation object 
seg = cloud_filtered.make_segmenter()
##Set the model you wish to fit 
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)
##Max distance for a point to be considered fitting the model 
max_distance = .01
seg.set_distance_threshold(max_distance)
##Call the segment function to obtain set of inlier indices and model coefficients 
inliers, coefficients = seg.segment()
# Extract inliers
extracted_inliers = cloud_filtered.extract(inliers, negative = True)
filename = 'extracted_inliers.pcd'
pcl.save(extracted_inliers, filename)
# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers
extracted_outliers = cloud_filtered.extract(inliers, negative = True)
filename = 'extracted_outliers.pcd'
pcl.save(extracted_outliers, filename)

# Save pcd for tabletop objects

