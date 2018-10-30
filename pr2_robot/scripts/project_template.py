#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {"test_scene_num": test_scene_num.data,
                 "arm_name": arm_name.data,
                 "object_name": object_name.data,
                 "pick_pose": message_converter.convert_ros_message_to_dictionary(pick_pose),
                 "place_pose": message_converter.convert_ros_message_to_dictionary(place_pose)}
    return yaml_dict


# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


# Callback function for your Point Cloud Subscriber
def pcl_callback(ros_cloud):

    # Convert ROS msg to PCL data
    pcl_cloud = ros_to_pcl(ros_cloud)

    # Statistical Outlier Filtering
    outlier_filter = pcl_cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(1)
    x = 0.1
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()
    filtered_pub.publish(pcl_to_ros(cloud_filtered))

    # Voxel Grid Downsampling
    vox = cloud_filtered.make_voxel_grid_filter()
    leaf_size = 0.01
    vox.set_leaf_size(leaf_size, leaf_size, leaf_size)
    cloud_filtered = vox.filter()

    # PassThrough Filter (z)
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # PassThrough Filter (y)
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.5
    axis_max = 0.5
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliners, coefficients = seg.segment()

    # Extract objects
    cloud_objects = cloud_filtered.extract(inliners, negative=True)

    # Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(10000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Classify the clusters!
    detected_objects_labels = []
    detected_objects = []

    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, index in enumerate(indices):
            color_cluster_point_list.append([white_cloud[index][0],
                                             white_cloud[index][1],
                                             white_cloud[index][2],
                                             rgb_to_float(cluster_color[j])])
        pcl_cluster = cloud_objects.extract(indices)

        # Convert the cluster from pcl to ROS using helper function
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features
        chists = compute_color_histograms(ros_cluster, using_hsv=False)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))
        detected_objects_labels.append([feature, str(j)])

        # Make the prediction, retrieve the label for the result and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[indices[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, j))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

        # Publish the list of detected objects
        detected_objects_pub.publish(detected_objects)

    # Create clusters based on color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    cluster_cloud_ros = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    objects_pub.publish(ros_cloud_objects)
    cluster_pub.publish(cluster_cloud_ros)

    # Check for correct number of detected objects and call mover
    if (scene_num == 1 and len(detected_objects) == 3 or
            scene_num == 2 and len(detected_objects) == 5 or
            scene_num == 3 and len(detected_objects) == 8):
        try:
            pr2_mover(detected_objects)
        except rospy.ROSInterruptException:
            pass


# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Get object labels and centroids
    labels = []
    centroids = []
    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append([np.asscalar(x) for x in np.mean(points_arr, axis=0)[:3]])

    # Loop through the pick list
    dict_list = []
    for i in range(len(object_list_param)):

        # Test score number
        test_scene_num.data = scene_num

        # Object name
        object_name.data = object_list_param[i]['name']

        # Arm name
        arm_name.data = 'right' if object_list_param[i]['group'] == 'green' else 'left'

        # Pick pose
        for j in range(len(labels)):
            if labels[j] == object_list_param[i]['name']:
                pick_pose.position.x = centroids[i][0]
                pick_pose.position.y = centroids[i][1]
                pick_pose.position.z = centroids[i][2]
                break

        # Place pose
        for j in range(len(dropbox_param)):
            if dropbox_param[j]['group'] == object_list_param[i]['group']:
                place_position = dropbox_param[j]['position']
                place_pose.position.x = place_position[0]
                place_pose.position.y = place_position[1]
                place_pose.position.z = place_position[2]

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # output yaml files
        send_to_yaml('output_{}.yaml'.format(scene_num), dict_list)


if __name__ == '__main__':

    # Set scene number
    scene_num = 1

    # Initialize color_list
    get_color_list.color_list = []

    # ROS node initialization
    rospy.init_node('perception', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    filtered_pub = rospy.Publisher('/filtered', PointCloud2, queue_size=1)
    cluster_pub = rospy.Publisher('/clusters', PointCloud2, queue_size=1)
    objects_pub = rospy.Publisher('/objects', PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers', Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects', DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model3.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
