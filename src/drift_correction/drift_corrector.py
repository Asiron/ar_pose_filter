#!/usr/bin/env python
#ROS Manifest for ARPose ros_build
import roslib
roslib.load_manifest('ar_pose')

#Python tools
from collections import defaultdict
from threading import RLock
import math

#ROS dependencies
import rospy, tf, roslib
import numpy as np
import tf.transformations as tfmath

#Messages and Services
from std_srvs.srv import Empty

from ar_pose.msg import ARMarkers
from geometry_msgs.msg import (Point, Quaternion)

class Util():

    @staticmethod
    def matrix_from_pose(tran, rot):
        translation_matrix = tfmath.translation_matrix(tran)
        quaternion_matrix  = tfmath.quaternion_matrix(rot)
        return np.dot(translation_matrix, quaternion_matrix)

    @staticmethod
    def matrix_from_pose_msg(pose):
        translation = Util.tuple_from_vector(pose.position)
        orientation = Util.tuple_from_quat(pose.orientation)
        return Util.matrix_from_pose(translation, orientation)

    @staticmethod
    def tuple_from_vector(vector):
        return (vector.x, vector.y, vector.z)

    @staticmethod
    def tuple_from_quat(quat):
        return (quat.x, quat.y, quat.z, quat.w)

    @staticmethod
    def distance(p0, p1):
        return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2 + (p0[2] - p1[2])**2)

    @staticmethod
    def median_alist(alist):
        sorted_list = sorted(alist)
        length = len(sorted_list)
        if not length % 2:
            return (sorted_list[length/2]+sorted_list[length/2-1])/2.0
        else:
            return (sorted_list[length/2])  

    @staticmethod
    def avg_alist(alist):
        return reduce(lambda x, y: x + y, alist) / len(alist)

    @staticmethod
    def apply_func(translations, func):
        xs, ys, zs = zip(*translations)
        return func(xs), func(ys), func(zs)

class PoseFilter():

    sample_size  = 10

    max_retries = 3

    median_yaw_treshold = 0.05
    median_x_treshold   = 0.2
    median_y_treshold   = 0.2
    median_z_treshold   = 0.2

    #For catching frames that do not locate Nao on the ground
    ground_treshold = 0.4

    #For catching frames that do not locate Nao's feet parallel to the ground (in radians)
    ground_roll_treshold  = 0.2
    ground_pitch_treshold = 0.2 

    buffer_lock = None
    frame_buffer = None
    buffer_latched = True

    def __init__(self, sample_size):
        self.sample_size = sample_size
        self.frames_buffer = defaultdict(list)
        self.buffer_lock = RLock()

    def locate(self):
        attemps = 1
        located = False
        localized_pose = None
        self.buffer_latched = False

        while not located:
            if attemps > self.max_retries:
                break
            self.clear_buffer()
            self.wait_for_buffer()
            localized_pose = self.apply_filter()
            if localized_pose != None:
                located = True    
            attemps += 1

        self.buffer_latched = True
        return localized_pose

    def wait_for_buffer(self):
        r = rospy.Rate(10) # 10Hz
        while not self.is_buffer_ready():
            r.sleep()              

    def is_buffer_ready(self):
        self.buffer_lock.acquire(blocking=True)
        ready = False
        for current_marker in self.frames_buffer.values():
            if len(current_marker) >= self.sample_size:
                ready = True
        
        self.buffer_lock.release()
        return ready

    def clear_buffer(self):
        self.buffer_lock.acquire(blocking=True)

        for k,v in self.frames_buffer.items():
            rospy.loginfo("Removing buffer: %s", k)
            del v[:]

        self.buffer_lock.release()

    def add_frame(self, marker_name, frame):
        self.buffer_lock.acquire(blocking=True)

        if not self.buffer_latched:
            self.frames_buffer[marker_name].append(frame)

        self.buffer_lock.release()

    def apply_filter(self):
        self.buffer_lock.acquire(blocking=True)
        self.drop_impossible_frames()
        rospy.loginfo("Applying filter")
        
        for marker_name, frames in self.frames_buffer.items():
            rospy.loginfo("  Processing marker: %s", marker_name)

            self.median_filter(marker_name, self.frames_buffer[marker_name])
            self.distance_filter(marker_name, self.frames_buffer[marker_name])
            self.distance_filter(marker_name, self.frames_buffer[marker_name])

        self.buffer_lock.release()

    def distance_filter(self, marker_name, frames):
        if len(frames) == 0:
            return

        translations = [x[0] for x in frames]
        yaws = [f[1][2] for f in frames]

        avg_point = Util.apply_func(translations, Util.avg_alist)
        avg_yaw = Util.avg_alist(yaws)
        
        rospy.loginfo("  {0} Averages: yaw: {1:.3f} x: {2:.3f} y: {3:.3f} z: {4:.3f}".format(marker_name, avg_yaw, *avg_point))
        distances = [Util.distance(avg_point, point) for point in translations]     
        avg_distance = Util.avg_alist(distances)

        current_marker = self.frames_buffer[marker_name]
        current_marker = [list1 + [distance] for list1, distance in zip(current_marker, distances)]

        current_marker[:] = [f for f in current_marker if f[3] < avg_distance]

        self.frames_buffer[marker_name] = current_marker

        for mar in self.frames_buffer[marker_name]:
            print mar[0], mar[3]

    def median_filter(self, marker_name, frames):
        if len(frames) == 0:
            return
        
        translations = [x[0] for x in frames]
        yaws = [f[1][2] for f in frames]

        median_point = Util.apply_func(translations, Util.median_alist)
        median_yaw = Util.median_alist(yaws)
        
        medians = list(median_point) + [median_yaw]

        current_marker = self.frames_buffer[marker_name]
        current_marker[:] = [f for f in current_marker if self.determine_median_validity(f, medians)]

    def drop_impossible_frames(self):
        rospy.loginfo("Dropping impossible frames")
        for marker_name in self.frames_buffer.keys():
            rospy.loginfo("  Processing marker: %s", marker_name)
            current_marker = self.frames_buffer[marker_name]
            current_marker[:] = [f for f in current_marker if self.determine_map_validity(f)]

    def determine_median_validity(self, frame, medians):
        if abs(medians[0] - frame[0][0]) > self.median_x_treshold:
            rospy.loginfo("\tOver median x treshold")
            return False
        elif abs(medians[1] - frame[0][1]) > self.median_y_treshold:
            rospy.loginfo("\tOver median y treshold")
            return False
        elif abs(medians[2] - frame[0][2]) > self.median_z_treshold:
            rospy.loginfo("\tOver median z treshold")
            return False
        elif abs(medians[3] - frame[1][2]) > self.median_yaw_treshold:
            rospy.loginfo("\tOver median yaw treshold")
            return False
        else:
            return True          

    def determine_map_validity(self, frame):
        if abs(frame[0][2]) > self.ground_treshold:
            rospy.loginfo("\tOver ground positional treshold: {0:3f}".format(frame[0][2]))
            return False
        elif abs(frame[1][0]) > self.ground_roll_treshold:
            rospy.loginfo("\tOver ground roll treshold: {0:3f}".format(frame[1][0]))
            return False
        elif abs(frame[1][1]) > self.ground_pitch_treshold:
            rospy.loginfo("\tOver ground pitch treshold: {0:3f}".format(frame[1][1]))
            return False
        else:
            return True

class DriftCorrection():

    #marker_names = [8,11,28,48,89]
    marker_names = [28]

    odom_frame_id   = "odom"
    camera_frame_id = "CameraTop_frame"
    base_footprint_frame_id = "base_footprint"

    tf_broadcaster = None
    tf_listener    = None
    marker_static_tranformations = defaultdict(tfmath.identity_matrix) 

    localize_srv_server = None

    pose_filter = None

    def __init__(self):
        self.pose_filter = PoseFilter(10)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener    = tf.TransformListener()
        self.marker_names   = ["4x4_" + str(marker_name) for marker_name in self.marker_names]

        self.localize_srv_server = rospy.Service("localize_nao", Empty, self.handle_localization)

    def start(self):
        self.get_static_marker_transforms()
        rospy.Subscriber("ar_pose_marker", ARMarkers, self.marker_callback, queue_size=1)

        self.pose_filter.buffer_latched = False
        self.pose_filter.wait_for_buffer()
        self.pose_filter.apply_filter()
        self.pose_filter.buffer_latched = True

        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():      
            r.sleep()

    def get_static_marker_transforms(self):
        for marker_name in self.marker_names:
            rospy.loginfo("Loading static transform for marker %s", marker_name)
            self.tf_listener.waitForTransform("map", marker_name, rospy.Time(), rospy.Duration(4.0))
            tran, rot = self.tf_listener.lookupTransform("map", marker_name, rospy.Time(0))
            rospy.loginfo("Static TF: {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *tran))
            self.marker_static_tranformations[marker_name] = Util.matrix_from_pose(tran, rot)

    def get_transform(self, from_frame, to_frame):
    	trans, rot = (None,None)
        self.tf_listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(from_frame, to_frame, now, rospy.Duration(4.0))
            trans, rot = self.tf_listener.lookupTransform(from_frame, to_frame, now)
        except (tf.Exception) as e:
            rospy.logwarn("Error while getting runtime transform \n %s", e)
        return Util.matrix_from_pose(trans, rot)

    def marker_callback(self, data):

        if len(data.markers) < 1:
            rospy.loginfo("No markers detected")
            return

        for marker in data.markers:

            marker_name = marker.header.frame_id

            if marker_name not in self.marker_static_tranformations.keys():
                continue

            marker_tf        = Util.matrix_from_pose_msg(marker.pose.pose)
            inv_marker_tf    = tfmath.inverse_matrix(marker_tf)
            map_to_camera_tf = np.dot(self.marker_static_tranformations[marker_name], inv_marker_tf)
            
            camera_to_footprint_tf = self.get_transform(self.camera_frame_id, self.base_footprint_frame_id)
            odom_to_footprint_tf   = self.get_transform(self.odom_frame_id, self.base_footprint_frame_id)

            map_to_footprint_tf = np.dot(map_to_camera_tf, camera_to_footprint_tf)
            
            map_to_footprint_translation = tfmath.translation_from_matrix(map_to_footprint_tf)
            map_to_footprint_rotation    = tfmath.euler_from_matrix(map_to_footprint_tf)

            rospy.loginfo("Foot Tran {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *map_to_footprint_translation))
            rospy.loginfo("Foot Rot  {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *map_to_footprint_rotation))

            self.pose_filter.add_frame(marker_name, [map_to_footprint_translation, map_to_footprint_rotation, odom_to_footprint_tf])

    def handle_localization(self, request):
        rospy.loginfo("Received localization service request")

if __name__ == '__main__':

    rospy.init_node('drift_correction')
    dc = DriftCorrection()
    dc.start()

