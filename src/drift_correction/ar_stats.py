#!/usr/bin/env python
import roslib
roslib.load_manifest('ar_pose')

from collections import defaultdict

import rospy, tf, roslib
import tf.transformations as tfmath
from ar_pose.msg import ARMarkers

def make_tuple_from_quat(quat):
    return [quat.x, quat.y, quat.z, quat.w]

class ARPoseStatistics():

    marker_statistics = defaultdict(int)

    time_at_beginning = None
    time_at_end       = None

    def __init__(self):
        rospy.Subscriber("markers_filtered/map_to_footprint", ARMarkers, self.marker_callback, queue_size=1)

    def marker_callback(self, data):

        if self.time_at_beginning == None:
            self.time_at_beginning = rospy.Time.now()

        print "-----------"
        for marker in data.markers:
            marker_name = marker.header.frame_id
            self.marker_statistics[marker_name] += 1
            self.print_marker(marker)

    def print_marker(self, marker):
        pose = marker.pose.pose
        position = pose.position
        rotation_= pose.orientation
        rotation_euler = tfmath.euler_from_quaternion(make_tuple_from_quat(rotation_))

        print marker.header.frame_id
        print "\t{0:.3f} {1:.3f} {2:.3f}".format(position.x, position.y, position.z)
        print "\t{0:.3f} {1:.3f} {2:.3f}".format(rotation_euler[0], rotation_euler[1], rotation_euler[2]) 
        print "\t{0:d}".format(marker.confidence)

    def finish_statistics(self):
        self.time_at_end = rospy.Time.now()
        print "Finished getting data from ar_pose"
        duration = self.time_at_end - self.time_at_beginning
        print "Total time elapsed: ", duration.secs, "s ", duration.nsecs/1000000, "ms"
        for k,v in self.marker_statistics.items():
            print "Marker ", k, " seen ", v, " times.", "({0:.2f} Hz)".format(v/duration.to_sec())


if __name__ == '__main__':

    rospy.init_node('ar_pose_stats')
    ar_stats = ARPoseStatistics()
    rospy.on_shutdown(ar_stats.finish_statistics)
    rospy.spin()