#!/usr/bin/env python
#ROS Manifest for ARPose ros_build
import roslib
roslib.load_manifest('ar_pose')

#Python tools
from collections import defaultdict, namedtuple
from threading import RLock
import math, operator, random

#ROS dependencies
import rospy, tf, roslib
import numpy as np
import tf.transformations as tfmath

from std_msgs.msg import Header

from ar_pose.msg import (ARMarkers, ARMarker)
from geometry_msgs.msg import (Point, Quaternion)

import util

class PoseFilter():

    #For catching frames that do not locate Nao on the ground
    ground_treshold = 0.4

    #For catching frames that do not locate Nao's feet parallel to the ground (in radians)
    ground_roll_treshold  = 0.2
    ground_pitch_treshold = 0.2

    def drop_impossible_frames(self, frames):
        rospy.loginfo("  Dropping impossible frames")  
        return [f for f in frames if self.determine_map_validity(f)]

    def determine_map_validity(self, frame):
        marker_name = frame[3]

        if abs(frame[0][2]) > self.ground_treshold:
            rospy.loginfo("\t{0} Over ground positional treshold: {1:3f}".format(marker_name, frame[0][2]))
            return False
        elif abs(frame[1][0]) > self.ground_roll_treshold:
            rospy.loginfo("\t{0} Over ground roll treshold: {1:3f}".format(marker_name, frame[1][0]))
            return False
        elif abs(frame[1][1]) > self.ground_pitch_treshold:
            rospy.loginfo("\t{0} Over ground pitch treshold: {1:3f}".format(marker_name, frame[1][1]))
            return False
        else:
            return True

class MarkerFilter():

    marker_names = [8,11,28,48,89]

    map_frame_id    = "map"
    odom_frame_id   = "odom"
    camera_frame_id = "CameraTop_frame"
    base_footprint_frame_id = "base_footprint"

    tf_broadcaster = None
    tf_listener    = None
    marker_static_tranformations = defaultdict(tfmath.identity_matrix) 

    filtered_camera_pose_pub    = None
    filtered_footprint_pose_pub = None

    pose_filter = None

    def __init__(self):
        self.pose_filter = PoseFilter()

        self.filtered_camera_pose_pub    = rospy.Publisher("markers_filtered/camera_to_marker", ARMarkers)
        self.filtered_footprint_pose_pub = rospy.Publisher("markers_filtered/map_to_footprint", ARMarkers)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener    = tf.TransformListener()
        self.marker_names   = ["4x4_" + str(marker_name) for marker_name in self.marker_names]

    def start(self):
        self.get_static_marker_transforms()
        rospy.Subscriber("ar_pose_marker", ARMarkers, self.marker_callback, queue_size=1)
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():     
            r.sleep()

    def get_static_marker_transforms(self):
        for marker_name in self.marker_names:
            rospy.loginfo("Loading static transform for marker %s", marker_name)
            self.tf_listener.waitForTransform(self.map_frame_id, marker_name, rospy.Time(), rospy.Duration(4.0))
            tran, rot = self.tf_listener.lookupTransform(self.map_frame_id, marker_name, rospy.Time(0))
            rospy.loginfo("Static TF: {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *tran))
            self.marker_static_tranformations[marker_name] = util.matrix_from_pose(tran, rot)

    def get_transform(self, from_frame, to_frame):
    	trans, rot = (None,None)
        self.tf_listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(from_frame, to_frame, now, rospy.Duration(4.0))
            trans, rot = self.tf_listener.lookupTransform(from_frame, to_frame, now)
        except (tf.Exception) as e:
            rospy.logwarn("Error while getting runtime transform \n %s", e)
        return util.matrix_from_pose(trans, rot)

    def construct_marker_msg(self, header, conf, pos, rot):
        marker = ARMarker()
        marker.header     = header
        marker.confidence = conf
        marker.pose.pose.position    = pos
        marker.pose.pose.orientation = rot
        return marker

    def publish_empty_markers(self):
        raw_markers = ARMarkers()
        footprint_poses = ARMarkers()
    
        self.filtered_camera_pose_pub.publish(raw_markers)
        self.filtered_footprint_pose_pub.publish(footprint_poses)

    def republish_filtered(self, filtered):
        
        if not filtered:
            self.publish_empty_markers()
            return

        raw_markers = ARMarkers()
        footprint_poses = ARMarkers()
        
        for f in filtered:

            conf = f[4]
            marker_name = f[3]
            cam_to_mar_pose = util.pose_from_matrix(f[2])
            cam_to_mar_pose_msg = util.pose_msg_from_pose(cam_to_mar_pose)
            map_to_footprint_pose_msg = util.pose_msg_from_pose((f[0], tfmath.quaternion_from_euler(*f[1])))

            header = Header()
            header.frame_id = marker_name
            header.stamp    = rospy.Time.now()

            orig_marker    = self.construct_marker_msg(header, conf, cam_to_mar_pose[0],
                                                                     cam_to_mar_pose[1])
            footprint_pose = self.construct_marker_msg(header, conf, map_to_footprint_pose_msg[0],
                                                                     map_to_footprint_pose_msg[1])

            raw_markers.markers.append(orig_marker)
            footprint_poses.markers.append(footprint_pose)

            self.tf_broadcaster.sendTransform(cam_to_mar_pose[0],
                                              cam_to_mar_pose[1],
                                              rospy.Time.now(),
                                              "camera_to_" + marker_name,
                                              marker_name)

        self.filtered_camera_pose_pub.publish(raw_markers)
        self.filtered_footprint_pose_pub.publish(footprint_poses)

    def marker_callback(self, data):

        if len(data.markers) < 1:
            self.publish_empty_markers()
            #rospy.loginfo("No markers detected")
            return

        frames_batch = []

        for marker in data.markers:

            conf = marker.confidence
            marker_name = marker.header.frame_id

            if marker_name not in self.marker_static_tranformations.keys():
                continue

            marker_tf        = util.matrix_from_pose_msg(marker.pose.pose)
            inv_marker_tf    = tfmath.inverse_matrix(marker_tf)
            map_to_camera_tf = np.dot(self.marker_static_tranformations[marker_name], inv_marker_tf)
            
            camera_to_footprint_tf = self.get_transform(self.camera_frame_id, self.base_footprint_frame_id)

            map_to_footprint_tf = np.dot(map_to_camera_tf, camera_to_footprint_tf)
            

            map_to_footprint_translation = tfmath.translation_from_matrix(map_to_footprint_tf)
            map_to_footprint_rotation    = tfmath.euler_from_matrix(map_to_footprint_tf)

            #rospy.loginfo("Foot Tran {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *map_to_footprint_translation))
            #rospy.loginfo("Foot Rot  {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *map_to_footprint_rotation))

            frames_batch.append((map_to_footprint_translation, map_to_footprint_rotation, inv_marker_tf, marker_name, conf))

        filtered_frames = self.pose_filter.drop_impossible_frames(frames_batch)
        self.republish_filtered(filtered_frames)

if __name__ == '__main__':

    rospy.init_node('marker_filter')
    mf = MarkerFilter()
    mf.start()

