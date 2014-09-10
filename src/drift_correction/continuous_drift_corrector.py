#!/usr/bin/env python
#ROS Manifest for ARPose ros_build
import roslib
roslib.load_manifest('ar_pose')

#Python tools
from collections import defaultdict, namedtuple
import math, operator, random

#ROS dependencies
import rospy, tf, roslib
import numpy as np
import tf.transformations as tfmath

from ar_pose.msg import ARMarkers
from geometry_msgs.msg import (Point, Quaternion)

import util

class MarkerInfo():

    freq = 0.0

    count = 0
    last_timestamp  = rospy.Time(0)
    reset_timestamp = rospy.Time(0) 

    drift_cor_data = None

    def __init__(self):
        self.count = 0
        self.last_timestamp  = rospy.Time(0)
        self.reset_timestamp = rospy.Time.now()

    def reset(self):
        self.count = 0
        self.odom_to_footprint_tf = None
        self.reset_timestamp = rospy.Time.now()

    def tick(self, drift_cor_data):
        self.count += 1
        self.drift_cor_data = drift_cor_data
        self.last_timestamp = rospy.Time.now()

    def check(self, timeout):
        if (rospy.Time.now() - self.last_timestamp).to_sec() > timeout :
            self.reset()

    def update_freq(self):
        if self.count < 3:
            self.freq = 0.0
        else:
            self.freq =  self.count / (rospy.Time.now() - self.reset_timestamp).to_sec()
        return self.freq

    def get_freq(self):
        return self.freq

class ContDriftCorrector():

    marker_names = [8,11,28,48,89]
    #marker_names = [28]
    
    timeout = 3
   
    '''
    max_lin_vel = 0.1361253808071727
    max_ang_vel = 0.7150091718968012
    '''

    max_distance   = 1.5   # meters
    max_angle_diff = 1.57  # rad

    test_time_diff = 1.0

    map_frame_id    = "map"
    odom_frame_id   = "odom"
    camera_frame_id = "CameraTop_frame"
    base_footprint_frame_id = "base_footprint"

    tf_listener    = None
    tf_broadcaster = None

    markers_infos = defaultdict(MarkerInfo)

    current_drift_correction    = None
    current_map_to_footprint_tf = None
    last_update_timestamp = rospy.Time(0)

    def __init__(self):
        self.tf_listener    = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.marker_names   = ["4x4_" + str(marker_name) for marker_name in self.marker_names]

    def start(self):
        rospy.Subscriber("markers_filtered/map_to_footprint", ARMarkers, self.marker_callback, queue_size=1)
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():     
            self.broadcast_correction()
            r.sleep()

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

    def broadcast_correction(self):
        if self.current_drift_correction == None:
            return
        self.tf_broadcaster.sendTransform(self.current_drift_correction[0],
                                  self.current_drift_correction[1],
                                  rospy.Time.now(),
                                  self.odom_frame_id,
                                  self.map_frame_id)

    def update_drift(self, location_mat, inv_odom_mat):
        drift_correction_mat = np.dot(location_mat, inv_odom_mat)
        self.current_drift_correction    = util.pose_from_matrix(drift_correction_mat)
        self.current_map_to_footprint_tf = location_mat
        self.last_update_timestamp = rospy.Time.now()

    def decide_marker(self):
        sorted_markers = sorted(self.markers_infos.items(), key=lambda x: x[1].update_freq(), reverse=True)
        
        print [(k,v.get_freq()) for (k,v) in sorted_markers]    

        chosen_marker = None

        for marker in sorted_markers:

            next_footprint_tf     = marker[1].drift_cor_data[0]
            previous_footprint_tf = self.current_map_to_footprint_tf
            
            #first marker caugth ever
            if previous_footprint_tf == None:
                previous_footprint_tf = next_footprint_tf

            change_tf = np.dot(next_footprint_tf, tfmath.inverse_matrix(previous_footprint_tf))
            change_tf_tran, change_tf_rot = util.tran_and_euler_from_matrix(change_tf)
            time_diff = (rospy.Time.now() - self.last_update_timestamp).to_sec()

            distance   = util.length(change_tf_tran)
            angle_diff = change_tf_rot[2]

            rospy.loginfo("Looking at marker %s", marker[0])
            rospy.loginfo("\t{0:.3f} m, {1:.3f} radians".format(distance, angle_diff))

            '''
            lin_velocity = distance / time_diff
            ang_velocity = angle_diff / time_diff

            rospy.loginfo("\t{0:.3f} > {1:.3f}, diff: {2:.3f}".format(lin_velocity, self.max_lin_vel, lin_velocity-self.max_lin_vel))
            rospy.loginfo("\t{0:.3f} > {1:.3f}, diff: {2:.3f}".format(ang_velocity, self.max_ang_vel, ang_velocity-self.max_ang_vel))
            '''

            if time_diff < self.test_time_diff:
                rospy.loginfo("Time check less than test time, checking vels")
                if distance > self.max_distance:
                    rospy.loginfo("\t Surpassing max distance in short time")
                    del self.markers_infos[marker[0]]
                    continue
                elif angle_diff > self.max_angle_diff:
                    rospy.loginfo("\t Surpassing max angular diff in short time")
                    del self.markers_infos[marker[0]]
                    continue

            # we have the marker, escape loop
            chosen_marker = marker
            break

        if chosen_marker == None:
            rospy.loginfo("No winning marker!")
            return

        rospy.loginfo("Winning marker %s %f", chosen_marker[0], chosen_marker[1].get_freq())
        self.update_drift(*chosen_marker[1].drift_cor_data)         

    def marker_callback(self, data):

        map(lambda x: x.check(self.timeout), self.markers_infos.values())

        if len(data.markers) < 1:
            #rospy.loginfo("No markers detected")
            return

        for marker in data.markers:

            marker_name = marker.header.frame_id

            if marker_name not in self.marker_names:
                continue

            basefootprint_to_odom_tf = self.get_transform(self.base_footprint_frame_id, self.odom_frame_id)
            map_to_basefootprint_tf  = util.matrix_from_pose_msg(marker.pose.pose)

            drift_cor_data = (map_to_basefootprint_tf, basefootprint_to_odom_tf)

            self.markers_infos[marker_name].tick(drift_cor_data)

        self.decide_marker()

if __name__ == '__main__':

    rospy.init_node('drift_correction')
    cdc = ContDriftCorrector()
    cdc.start()

