#!/usr/bin/env python

# Import modules
from pcl_helper import *

# TODO: Define functions as required

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(20)
    x = 0.3
    outlier_filter.set_std_dev_mul_thresh(x)
    pcl_cloud = outlier_filter.filter()
    # TODO: Voxel Grid Downsampling
    vox = pcl_cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()
    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.3
    passthrough.set_filter_limits (axis_min, axis_max)
    cloud_passthrough = passthrough.filter()
    passthrough = cloud_passthrough.make_passthrough_filter()
    
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits (axis_min, axis_max)
    cloud_passthrough = passthrough.filter()
    # TODO: RANSAC Plane Segmentation
    seg = cloud_passthrough.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()
    # TODO: Extract inliers and outliers
    cloud_objects = cloud_passthrough.extract(inliers, negative=True)
    cloud_table = cloud_passthrough.extract(inliers, negative=False)
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(25)
    ec.set_MaxClusterSize(2800)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('object_recognition', anonymous=True)
    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size = 1)
    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size = 1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size = 1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size = 1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects",DetectedObjectsArray, queue_size=1)
    # Initialize color_list
    get_color_list.color_list = []
    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
