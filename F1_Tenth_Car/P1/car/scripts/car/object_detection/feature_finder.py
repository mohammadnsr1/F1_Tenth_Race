from distutils.command.config import config
from sklearn.cluster import DBSCAN
import numpy as np
import cv2 as cv
import time
import car.object_detection.constants as constants
import math
class FeatureTracker:
    tracked_objects = []
    def __init__(self):
        self.orb = cv.ORB_create()
        # for x in range(12):
        #     sample_image = cv.imread("src/car_tracker/images/sample"+str(x+1)+".png", 0) # trainImage
        #     query_keypoints, query_descriptors = self.orb.detectAndCompute(sample_image,None)
        #     tracked_objecct = (query_keypoints,query_descriptors)
        #     self.tracked_objects.append(tracked_objecct)
        self.MIN_MATCH_COUNT = 10

    def get_corners(self,image):
        query_keypoints, query_descriptors = self.orb.detectAndCompute(image,None)
        corners = []

        for keypoint in query_keypoints:
            corners.append(np.asarray(keypoint.pt))
        corners_array = np.array(corners)
        return query_keypoints,query_descriptors,corners_array

    def get_clusters(self,corners):
        # was 20 min samp
        cluster_data = DBSCAN(eps=80, min_samples=20).fit(corners)
        labels = cluster_data.labels_
        return labels
        
    def get_matches(self,orb_keypoints, orb_description, clusters):
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks = 50)

        flann = cv.FlannBasedMatcher(index_params, search_params)

        # Loop through clusters and match
        unique_cluster = np.unique(clusters)
        unique_cluster = np.delete(unique_cluster,np.where(unique_cluster == -1),None)
        
        car_matches = []
        for cluster in unique_cluster:
            orb_descriptors_in_cluster = []
            orb_keypoints_in_cluster = []
            for index in range(len(orb_description)):
                if clusters[index] == cluster:
                    orb_descriptors_in_cluster.append(orb_description[index])
                    orb_keypoints_in_cluster.append(orb_keypoints[index])

            current_image_descriptors = np.float32(np.asarray(orb_descriptors_in_cluster))

            for track_obj_index,(query_keypoints, query_descriptors) in enumerate(self.tracked_objects):
                tracked_object_descriptors = np.float32(query_descriptors)

                matches = flann.knnMatch(current_image_descriptors, tracked_object_descriptors, 2)

                # store all the good matches as per Lowe's ratio test.
                good = []
                for m,n in matches:
                    if m.distance < 0.7*n.distance:
                        good.append(m)

                if len(good)>self.MIN_MATCH_COUNT:
                    src_pts = np.float32([ orb_keypoints_in_cluster[m.queryIdx].pt for m in good ])
                    x_min = constants.original_width_image
                    x_max = 0
                    y_min = constants.original_height_image
                    y_max = 0
                    for point in src_pts:
                        if point[0] > x_max:
                            x_max = point[0]
                        if point[1] > y_max:
                            y_max = point[1]
                        if point[0] < x_min:
                            x_min = point[0]
                        if point[1] < y_min:
                            y_min = point[1]
                    car_matches.append((x_min, x_max, y_min, y_max))
                    break
        return car_matches
