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

out_yaml_file = 'output_1.yaml'

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

	# Exercise-2 TODOs:

	    # TODO: Convert ROS msg to PCL data
	cloud = ros_to_pcl(pcl_msg)
	print(cloud)

	    # TODO: Statistical Outlier Filtering
	# we start by creating a filter object: 
	vox = cloud.make_voxel_grid_filter()
	outlier_filter = cloud.make_statistical_outlier_filter()
	outlier_filter.set_mean_k(3) # Set the number of neighboring points to analyze for any given point
	# Any point with a mean distance larger than global (mean distance+1*std_dev) will be considered outlier
	outlier_filter.set_std_dev_mul_thresh(0.) 
	cloud_filtered = outlier_filter.filter() # Finally call the filter function for magic
	pub_noise.publish(pcl_to_ros(cloud_filtered))

	    # TODO: Voxel Grid Downsampling
	vox = cloud_filtered.make_voxel_grid_filter()
	LEAF_SIZE = 0.01
	vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
	cloud_filtered = vox.filter()	

	    # TODO: PassThrough Filter
	passthrough = cloud_filtered.make_passthrough_filter()
	filter_axis = 'z'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = 0.61
	axis_max = 0.9
	passthrough.set_filter_limits(axis_min, axis_max)
	cloud_filtered = passthrough.filter()

	passthrough = cloud_filtered.make_passthrough_filter()
	filter_axis = 'y'
	passthrough.set_filter_field_name(filter_axis)
	axis_min = -0.5
	axis_max = 0.5
	passthrough.set_filter_limits(axis_min, axis_max)
	cloud_filtered = passthrough.filter()

	pub_passthrough.publish(pcl_to_ros(cloud_filtered))

	    # TODO: RANSAC Plane Segmentation
	seg = cloud_filtered.make_segmenter()
	seg.set_model_type(pcl.SACMODEL_PLANE)
	seg.set_method_type(pcl.SAC_RANSAC)
	max_distance = 0.005
	seg.set_distance_threshold(max_distance)
	inliers, coefficients = seg.segment()

	    # TODO: Extract inliers and outliers
	cloud_table = cloud_filtered.extract(inliers, negative = False)
	cloud_objects = cloud_filtered.extract(inliers, negative = True)
	pub_table.publish(pcl_to_ros(cloud_table))
	pub_table_outlier.publish(pcl_to_ros(cloud_objects))


	    # TODO: Euclidean Clustering
	white_cloud = XYZRGB_to_XYZ(cloud_objects)# Apply function to convert XYZRGB to XYZ
	tree = white_cloud.make_kdtree()
	# Create a cluster extraction object
	ec = white_cloud.make_EuclideanClusterExtraction()
	# Set tolerances for distance threshold as well as minimum and maximum cluster size (in points)
	ec.set_ClusterTolerance(0.05)
	ec.set_MinClusterSize(100)
	ec.set_MaxClusterSize(10000)
		# Search the k-d tree for clusters
	ec.set_SearchMethod(tree)
		# Extract indices for each of the discovered clusters
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

	    # TODO: Convert PCL data to ROS messages and publish
	ros_cloud = pcl_to_ros(cluster_cloud)
	points_pub.publish(ros_cloud)

	# Exercise-3 TODOs:
	# Classify the clusters! (loop through each detected cluster one at a time)
    	detected_objects_labels = []
    	detected_objects = []

	for index, pts_list in enumerate(cluster_indices):
		# Grab the points for the cluster from the extracted outliers (cloud_objects)
		pcl_cluster = cloud_objects.extract(pts_list)

		# convert the cluster from pcl to ROS using helper function
		ros_cluster = pcl_to_ros(pcl_cluster)

		# Extract histogram features
		chists = compute_color_histograms(ros_cluster, using_hsv=True)
            	normals = get_normals(ros_cluster)
            	nhists = compute_normal_histograms(normals)
            	feature = np.concatenate((chists, nhists))

		# Make the prediction, retrieve the label for the result
		# and add it to detected_objects_labels list
		prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
		label = encoder.inverse_transform(prediction)[0]
		detected_objects_labels.append(label)

		# Publish a label into RViz
		label_pos = list(white_cloud[pts_list[0]])
		label_pos[2] += .4
		object_markers_pub.publish(make_label(label,label_pos, index))

		# Add the detected object to the list of detected objects.
		do = DetectedObject()
		do.label = label
		do.cloud = ros_cluster
		detected_objects.append(do)



	    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
	    # Could add some logic to determine whether or not your object detections are robust
	    # before calling pr2_mover()
	try:
		pr2_mover(detected_objects)
	except rospy.ROSInterruptException:
		pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):
	# TODO: Initialize variables
	yaml_dict_list = []

	# TODO: Get/Read parameters
	object_list_param = rospy.get_param('/object_list')

	# TODO: Parse parameters into individual variables
#	for i, obj in enumerate(object_list_param)
#		object_names.append(obj['name'])
#		object_groups.append(obj['group'])


	# TODO: Rotate PR2 in place to capture side tables for the collision map

	# TODO: Loop through the pick list
	for i, obj in enumerate(object_list_param):

		# TODO: Get the PointCloud for a given object and obtain it's centroid
		labels = []
		centroids = [] # to be list of tuples (x, y, z)
		for obj_detected in object_list:
	    		labels.append(obj_detected.label)
	    		points_arr = ros_to_pcl(obj_detected.cloud).to_array()
	    		centroids.append(np.mean(points_arr, axis=0)[:3])

		if obj['name'] in labels:
			# check test scene number
			test_scene_num = Int32()
			#test_scene_num_param = rospy.get_param()
			test_scene_num.data = 1

			# write name of the object we search for
			object_name = String()
			object_name.data = obj['name']

	

			# TODO: Assign the arm to be used for pick_place
			arm_name = String()
			arm_name.data = 'left' if obj['group'] == 'red' else 'right'

			# TODO: Create 'place_pose' for the object
			place_pose = Pose()
			place_pose_param = rospy.get_param('/dropbox')
			place_pose.position.x, place_pose.position.y, place_pose.position.z = place_pose_param[0]['position'] if place_pose_param[0]['group'] == obj['group'] else place_pose_param[1]['position']

			# Create pick_pose
			pick_pose = Pose()
			item_index = labels.index(obj['name'])
			centroid = centroids[item_index]
			pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = centroid.tolist()


			# TODO: Create a dictionary (made with make_yaml_dict()) for later output to yaml format
			yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
			yaml_dict_list.append(yaml_dict)

			# Wait for 'pick_place_routine' service to come up
			rospy.wait_for_service('pick_place_routine')

			try:
			    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

			    # TODO: Insert your message variables to be sent as a service request
			    resp = pick_place_routine(test_scene_num, arm_name, object_name, pick_pose, place_pose)

			    print ("Response: ",resp.success)

			except rospy.ServiceException, e:
			    print "Service call failed: %s"%e
	

	# TODO: Output your request parameters into output yaml file
	send_to_yaml(out_yaml_file,yaml_dict_list)


if __name__ == '__main__':

	# TODO: ROS node initialization
	rospy.init_node('clustering', anonymous=True)

	# TODO: Create Subscribers
	pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

	# TODO: Create Publishers
	points_pub = rospy.Publisher("/points", PointCloud2, queue_size=1)
	pub_table = rospy.Publisher("/pub_table", PointCloud2, queue_size=1)
	pub_table_outlier = rospy.Publisher("/pub_table_outlier", PointCloud2, queue_size=1)
	pub_cluster = rospy.Publisher("/pub_cluster", PointCloud2, queue_size=1)
	pub_noise = rospy.Publisher("/pub_noise", PointCloud2, queue_size=1)
	pub_passthrough = rospy.Publisher("/pub_passthrough", PointCloud2, queue_size=1)
	object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
	detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)


	# TODO: Load Model From disk
	model = pickle.load(open('model.sav', 'rb'))
    	clf = model['classifier']
    	encoder = LabelEncoder()
    	encoder.classes_ = model['classes']
    	scaler = model['scaler']

	# Initialize color_list
	get_color_list.color_list = []

	# TODO: Spin while node is not shutdown
	while not rospy.is_shutdown():
		rospy.spin()
