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
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data using helper function
    cloud = ros_to_pcl(pcl_msg)

    # Statistical outlier filtering : Since we are dealing with noisy environment
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    x = 1.0
    outlier_filter.set_std_dev_mul_thresh(x)
    cloud_filtered = outlier_filter.filter()

    # Voxel Grid Downsampling : Reducing number of volume elements in the picture
    vox = cloud_filtered.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter : Extracting everything except for the table as outliers
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)

    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers : Extract indices filter
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    #Creating a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.001)
    ec.set_MinClusterSize(10)
    ec.set_MaxClusterSize(250)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j,indices in enumerate(cluster_indices):
        for i,indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0], white_cloud[indice][1],
                                             white_cloud[indice][2], rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # Publish ROS messages using helper function
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)


# Exercise-3:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects = []
    detected_objects_labels = []

    # Grab the points for the cluster
    for index, pts_list in enumerate(cluster_indices):
        pcl_cluster = cloud_objects.extract(pts_list)
        cloud_ros = pcl_to_ros(pcl_cluster)
        chists = compute_color_histograms(cloud_ros, using_hsv=False)
        normals = get_normals(cloud_ros)
        nhists = compute_normal_histograms(normals)

        # Compute the associated feature vector
        feature = np.concatenate((chists, nhists))
        labeled_features.append([feature, model_name])

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature,reshape(-1,1)))
        label = encoder.inverse_transform(prediction[0])
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    # Publish the list of detected objects
    rospy.loginfo('Detected {} objects : {}'.format(len(detected_objects_labels), detected_objects_labels))
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

def pr2_mover(object_list):

    # Initialize variables and ROS messages
    output_list = []
    # The appropriate message types were inferred from the srv file
    test_scene_num = Int32()
    object_name = String()
    object_group = None
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    # Get/Read parameters
    object_list_params = rospy.get_param("/object_list")
    dropbox_params = rospy.get_param("/dropbox")
    # Set the scene to 1
    test_scene_num.data = 1

    # Parse parameters into individual variables
    for param in object_list_params:
        object_name.data = param['name']
        object_group = param['group']

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    # Loop through the pick list

        for obj in object_list:
            # Get the PointCloud for a given object and obtain it's centroid
            centroid = []
            points_arr = ros_to_pcl(obj.cloud).to_array()
            centroids.append(np.mean(points_arr, axis=0)[:3])

            # Create 'pick_pose' for the object
            pick_pose.position.x = centroids[object][0]
            pick_pose.position.y = centroids[object][1]
            pick_pose.position.z = centroids[object][2]

        for params in dropbox_params:
            obj_position = params['position']
            # Create 'place_pose' for the object
            place_pose.position.x = obj_position[0]
            place_pose.position.y = obj_position[1]
            place_pose.position.z = obj_position[2]


        # Assign the arm to be used for pick_place
        if object_group =='green':
           arm_name = 'right'
        elif object_group == 'red':
           arm_name = 'left'

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        dict_list = []
        for i in range(0, len(object_list_params)):
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            dict_list.append(yaml_dict)


        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    send_to_yaml('output_{}.yaml'.format(test_scene_num), dict_list)

if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', PointCloud2, pcl_callback, queue_size=1)
    
    # Create Publishers
    pcl_objects_pub = rospy.Publisher('/pcl_objects',PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table',PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher('/object_markers',Marker, queue_size=1)


    # Load Model From disk
    model = pickle.load(open('model.sav','rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
