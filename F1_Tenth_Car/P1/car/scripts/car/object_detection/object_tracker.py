#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2 as cv
import torch
import car.object_detection.utils as utils
import car.object_detection.constants as constants
import car.object_detection.feature_finder as feature_finder
from car.object_detection.yolo_model import YOLO
from car.object_detection.pose_model import PoseModel
import cv_bridge
import math
import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
import geometry_msgs.msg
import tf.transformations
import time

device = "cuda" if torch.cuda.is_available() else "cpu"

yolo_model = YOLO()
yolo_model.load_state_dict(torch.load("/home/ryan/ryan_brown_ws/src/car/models/object_detection/yolo_trained_network.pth"))
yolo_model.to(device)
yolo_model.eval()

pose_model = PoseModel()
pose_model.load_state_dict(torch.load("/home/ryan/ryan_brown_ws/src/car/models/object_detection/pose_trained_network.pth"))
pose_model.to(device)
pose_model.eval()

bridge = cv_bridge.CvBridge()
image_pub = rospy.Publisher("test_bounding_boxes",Image)
pose_pub = rospy.Publisher("car_local_pointer",geometry_msgs.msg.PoseArray)
feature_tracker = feature_finder.FeatureTracker()
def learning_callback(data):
    with torch.no_grad():
        # rospy.loginfo("Test")
        cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
        # For YOLO
        resized_image = cv.resize(cv_image,(constants.yolo_width_of_image,constants.yolo_height_of_image))
        camera_tensor = torch.from_numpy(resized_image).to(device)
        removed_nan_tensor = torch.nan_to_num(camera_tensor.to(device), nan=100.0, posinf=100.0, neginf=100.0)
        normalized_input = removed_nan_tensor.mul(-1/5)
        # For Pose
        removed_nan_tensor_full = torch.nan_to_num(torch.from_numpy(cv_image).to(device), nan=100.0, posinf=100.0, neginf=100.0)
        normalized_input_full = removed_nan_tensor_full.mul(-1/5)

        network_prediction = yolo_model(normalized_input.unsqueeze(0).unsqueeze(0))
        bounding_boxes = utils.get_bounding_boxes_for_prediction(network_prediction)

        # Publishing Local Map
        pose_array = geometry_msgs.msg.PoseArray()
        pose_array.poses = []
        pose_topic_model = geometry_msgs.msg.Pose()
        point_topic_model = geometry_msgs.msg.Point()
        point_topic_model.x = 0
        point_topic_model.y = 0
        pose_topic_model.position = point_topic_model
        pose_array.poses.append(pose_topic_model)

        for bounding_box in bounding_boxes:
            #Display
            x_min = max(0,int(constants.original_width_image / constants.yolo_width_of_image *(bounding_box[1]-bounding_box[3]//2)))
            x_max = min(constants.original_width_image,int(constants.original_width_image / constants.yolo_width_of_image *(bounding_box[1]+bounding_box[3]//2)))
            y_min = max(0,int(constants.original_height_image / constants.yolo_height_of_image *(bounding_box[2]-bounding_box[4]//2)))
            y_max = min(constants.original_height_image,int(constants.original_height_image / constants.yolo_height_of_image *(bounding_box[2]+bounding_box[4]//2)))
            y_cap = int(y_max - 1/10 *(y_max-y_min))
            x_avg = int((x_min+x_max)/2)
            start_point = (x_min, y_min)
            end_point = (x_max, y_max)
            color = 9
            thickness = 2
            cv_image = cv.rectangle(cv_image, start_point, end_point, color, thickness)
            # Find Pose
            cropped_image = normalized_input_full[y_min:y_max, x_min:x_max].cpu()
            size_of_input_image = (constants.crop_image_resize_width, constants.crop_image_resize_height)
            scaled_cropped_image = cv.resize(cropped_image.numpy(), size_of_input_image, interpolation=cv.INTER_LINEAR)
            input_to_pose_network = torch.from_numpy(scaled_cropped_image).to(device).unsqueeze(0).unsqueeze(0)
            yaw = pose_model(input_to_pose_network).item()
            # Find X Y in local frame
            distance_box_to_consider = removed_nan_tensor_full[y_min:y_cap,x_avg]
            distance_to_center_of_box=  torch.min(distance_box_to_consider)
            index_of_min = torch.argmin(distance_box_to_consider)
            x_distance_in_pixel_plane = constants.original_width_image//2 - ((x_max+x_min)/2)
            # x_distance_in_pixel_plane = constants.original_width_image//2 - (x_min+col_of_min)
            depth_ray_in_pixel_frame = math.sqrt(pow(x_distance_in_pixel_plane,2) + pow(constants.camera_horizontal_focal_length,2))
            ratio_from_real_frame_to_pixel_frame = distance_to_center_of_box / depth_ray_in_pixel_frame
            y_local_frame = constants.camera_horizontal_focal_length * ratio_from_real_frame_to_pixel_frame
            x_local_frame = x_distance_in_pixel_plane * ratio_from_real_frame_to_pixel_frame

            pose_topic_model = geometry_msgs.msg.Pose()
            point_topic_model = geometry_msgs.msg.Point()
            point_topic_model.x = y_local_frame
            point_topic_model.y = x_local_frame
            orientation_model = geometry_msgs.msg.Quaternion()
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            orientation_model.w = quaternion[3]
            orientation_model.x = quaternion[0]
            orientation_model.y = quaternion[1]
            orientation_model.z = quaternion[2]
            pose_topic_model.orientation = orientation_model
            pose_topic_model.position = point_topic_model
            pose_array.poses.append(pose_topic_model)

        pose_array.header.frame_id = 'map'
        pose_pub.publish(pose_array)

    ret, reduced_range_image = cv.threshold(cv_image,10,100,cv.THRESH_TRUNC)
    image_pub.publish(bridge.cv2_to_imgmsg(reduced_range_image, "32FC1"))

def feature_tracking_callback(data):
    cv_image = bridge.imgmsg_to_cv2(data,desired_encoding='passthrough')
    non_nan_array = np.nan_to_num(cv_image, nan=100, posinf=100, neginf=100)
    cv_image = np.clip(non_nan_array,a_min=0,a_max=10)
    for row_index in range(len(cv_image)):
        row = cv_image[row_index]
        # Remove in final simulation
        # Simulates noise
        # row = row + np.random.normal(0,0.02 * row.max(),len(row))
        row = row / (row.max() + 5)
        row = row + row.min()
        row = row / row.max() * 255
        row = np.clip(row,0,255)
        cv_image[row_index] = row 

    reduced_dimension_image = np.uint8(cv_image)

    # cv.imwrite("src/car_tracker/images/testImage1.png", reduced_dimension_image)
    # Find Conrners

    query_keypoints, query_descriptors, corners_array = feature_tracker.get_corners(reduced_dimension_image)
    if len(corners_array)>0:
        # Get Clusters
        clusters = feature_tracker.get_clusters(corners_array)
        colors = (clusters + 1)
        colors = colors * 255 / max(1,colors.max())

        # Find Matches 
        object_matches = feature_tracker.get_matches(query_keypoints,query_descriptors,clusters)

        #Draw Results
        if len(clusters) > 0:
            key_point_image = output_image = cv.drawKeypoints(reduced_dimension_image, query_keypoints, 0, (0,0,0),flags=0)
            for corner_index, corner in enumerate(corners_array):
                output_image = cv.circle(output_image, (int(corner[0]),int(corner[1])), 5, (int(colors[corner_index]),int(colors[corner_index]),0), -1)
            for object in object_matches:
                first_point = (int(object[0]),int(object[2]))
                second_point = (int(object[1]),int(object[3]))
                color = (255,0,0)
                cv.rectangle(output_image,first_point,second_point,color,2)
                x_min= int(object[0])
                x_max= int(object[1])
                y_min= int(object[2])
                y_max= int(object[3])
                y_cap = int(y_max - 2/10 * (y_max-y_min))
                distance_to_center_of_box = non_nan_array[y_min:y_cap,x_min:x_max].min()
                x_distance_in_pixel_plane = constants.original_width_image//2 - ((x_max+x_min)/2)
                depth_ray_in_pixel_frame = math.sqrt(pow(x_distance_in_pixel_plane,2) + pow(constants.camera_horizontal_focal_length,2))
                ratio_from_real_frame_to_pixel_frame = distance_to_center_of_box / depth_ray_in_pixel_frame
                y_local_frame = constants.camera_horizontal_focal_length * ratio_from_real_frame_to_pixel_frame
                x_local_frame = x_distance_in_pixel_plane * ratio_from_real_frame_to_pixel_frame
                print("y_local_frame: "+str(y_local_frame)+" x_local_frame: "+ str(x_local_frame))

            image_pub.publish(bridge.cv2_to_imgmsg(output_image, "8UC3"))
        else:
            image_pub.publish(bridge.cv2_to_imgmsg(cv.Mat(reduced_dimension_image), "8UC1"))
    else:
            image_pub.publish(bridge.cv2_to_imgmsg(cv.Mat(reduced_dimension_image), "8UC1"))

def setup():
    rospy.init_node('object_tracker_estimation')
    rospy.Subscriber("/3d_image/image_raw_depth", Image, learning_callback)
    # rospy.Subscriber("/3d_image/image_raw_depth", Image, feature_tracking_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    setup()
