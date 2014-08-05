#!/usr/bin/env python
import roslib
roslib.load_manifest('ar_pose')

from collections import (namedtuple, defaultdict)

from operator import concat
import rospy, tf, roslib
import tf.transformations as tfmath
from ar_pose.msg import ARMarkers
from geometry_msgs.msg import (Point, Quaternion)

StaticTransform = namedtuple('StaticTransform', ['position', 'orientation'])

class DriftCorrection():

    markers = [1,2]
    broadcaster = None

    marker_static_tranformations = defaultdict(StaticTransform) 

    def __init__(self):
        broadcaster = tf.TransformBroadcaster()
        self.markers = ["4x4_" + str(marker) for marker in self.markers]

    def start(self):
        self.get_static_marker_transforms()

        print self.marker_static_tranformations

        rospy.Subscriber("ar_pose_marker", ARMarkers, self.marker_callback)


        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():      
            r.sleep()

    def get_static_marker_transforms(self):
        listener = tf.TransformListener()
        for marker in self.markers:
            try:
                listener.waitForTransform("/map", marker, rospy.Time(), rospy.Duration(4.0))
            except:
                pass

            try:
                now = rospy.Time.now()
                listener.waitForTransform("/map", marker, now, rospy.Duration(4.0))
                (trans, rot) = listener.lookupTransform("/map", marker, now)
                self.marker_static_tranformations[marker] = StaticTransform(position=trans, orientation=rot)
            except (tf.Exception):
                print "Error while getting static transformations"


    def make_tuple_from_point(self, point):
        return (point.x, point.y, point.z)

    def make_tuple_from_quat(self, quat):
        return (quat.x, quat.y, quat.z, quat.w)

    def mul_transformations(self, first_pose, second_pose):
        print first_pose
        first_matrix_vector = tfmath.translation_matrix(first_pose.position)
        first_matrix_quat   = tfmath.translation_matrix(first_pose.orientation)

        first_matrix = first_matrix_quat * first_matrix_vector 

        print first_matrix
        rospy.sleep(1000)

        second_matrix_vector = tfmath.translation_matrix(self.make_tuple_from_point(second_pose.position))
        second_matrix_quat   = tfmath.translation_matrix(self.make_tuple_from_quat(second_pose.orientation))

        second_matrix = second_matrix_quat * second_matrix_vector

        final_matrix = first_matrix * second_matrix

        return final_matrix

    def marker_callback(self, data):

        if len(data.markers) < 1:
            return

        for marker in data.markers:
            new_trans = self.mul_transformations(self.marker_static_tranformations[marker.header.frame_id],
                                     marker.pose.pose)
            
            print marker.header.frame_id, new_trans

if __name__ == '__main__':

    rospy.init_node('drift_correction')
    dc = DriftCorrection()
    dc.start()
