#!/usr/bin/env python
import roslib
roslib.load_manifest('ar_pose')

from collections import (namedtuple, defaultdict)

from operator import concat
import rospy, tf, roslib
import numpy as np
import tf.transformations as tfmath
from ar_pose.msg import ARMarkers
from geometry_msgs.msg import (Point, Quaternion)

Transform = namedtuple('Transform', ['position', 'orientation'])

class DriftCorrection():

    markers = [79,83,85,91,94]

    broadcaster = None
    listener    = None
    marker_static_tranformations = defaultdict(Transform) 

    current_drift_correction_transform = Transform(position=(0,0,0), orientation=(0,0,0,1))

    def __init__(self):
        self.broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        self.markers = ["4x4_" + str(marker) for marker in self.markers]

    def start(self):
        self.get_static_marker_transforms()

        print self.marker_static_tranformations

        rospy.Subscriber("ar_pose_marker", ARMarkers, self.marker_callback)


        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():      
            r.sleep()

    def get_static_marker_transforms(self):
        for marker in self.markers:
            try:
                self.listener.waitForTransform("/map", marker, rospy.Time(), rospy.Duration(4.0))
            except:
                pass

            try:
                now = rospy.Time.now()
                self.listener.waitForTransform("/map", marker, now, rospy.Duration(4.0))
                self.marker_static_tranformations[marker] = self.get_transform("/map", marker, now)
            except (tf.Exception):
                print "Error while getting static transformations"


    def make_tuple_from_point(self, point):
        return (point.x, point.y, point.z)

    def make_tuple_from_quat(self, quat):
        return [quat.x, quat.y, quat.z, quat.w]

    def create_matrix(self, vec, quat):
    	trans_matrix = tfmath.translation_matrix(vec)
    	quat_matrix  = tfmath.quaternion_matrix(quat)
    	return np.dot(trans_matrix, quat_matrix)

    def mul_transforms(self, first, second):
    	first_matrix  = self.create_matrix(first.position, first.orientation)
    	second_matrix = self.create_matrix(second.position, second.orientation)
    	final_matrix =  np.dot(first_matrix, second_matrix)
    	return Transform(position=tfmath.translation_from_matrix(final_matrix),
    					 orientation=tfmath.quaternion_from_matrix(final_matrix))

    def invert_transform(self, transform):
    	matrix = self.create_matrix(transform.position, transform.orientation)
    	inverted_matrix = tfmath.inverse_matrix(matrix)
    	return Transform(position=tfmath.translation_from_matrix(inverted_matrix),
    					 orientation=tfmath.quaternion_from_matrix(inverted_matrix))

    def get_transform(self, from_frame, to_frame, time):
    	trans, rot = (None,None)
        try:
            self.listener.waitForTransform(from_frame, to_frame, time, rospy.Duration(1))
            (trans, rot) = self.listener.lookupTransform(from_frame, to_frame, time)
        except (tf.Exception):
            print "Error while getting runtime transform"
        return Transform(position=trans, orientation=rot)

    def marker_callback(self, data):

        if len(data.markers) < 1:
            self.broadcast_drift_correction(rospy.Time.now())
            return

        chosen_marker = None
        for marker in data.markers:
        	if marker.header.frame_id in self.marker_static_tranformations.keys():
        		chosen_marker = marker

        if chosen_marker == None:
        	self.broadcast_drift_correction(rospy.Time.now())
        	return

        marker_to_camera_pos  = self.make_tuple_from_point(marker.pose.pose.position)
        marker_to_camera_quat = self.make_tuple_from_quat(marker.pose.pose.orientation)


        marker_to_camera = self.invert_transform(Transform(position=marker_to_camera_pos,
                                     orientation=marker_to_camera_quat))

        map_to_camera = self.mul_transforms(self.marker_static_tranformations[chosen_marker.header.frame_id], marker_to_camera)

        print "Looking at ", chosen_marker.header.frame_id

        now = rospy.Time.now()
        odom_to_camera = self.get_transform("/CameraTop_frame", "/odom", now)

        if odom_to_camera.position != None and odom_to_camera.orientation != None:
        	self.current_drift_correction_transform = self.mul_transforms(map_to_camera, odom_to_camera)
       	
        self.broadcast_drift_correction(now)

    def broadcast_drift_correction(self, time):
    	tran = self.current_drift_correction_transform
        self.broadcaster.sendTransform(tran.position,
							   tran.orientation,
                               time,
                               "/odom",
                               "/map")

if __name__ == '__main__':

    rospy.init_node('drift_correction')
    dc = DriftCorrection()
    dc.start()
