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

#Messages and Services
from std_srvs.srv import Empty

from ar_pose.msg import ARMarkers
from geometry_msgs.msg import (Point, Quaternion)

import util

SampleStat = namedtuple('SampleStat', ['dropped', 'initial', 'mad'])


class PoseFilter():
    '''
    Object that collects ARMarker frames and stores them in the dictionary with the key as a 
    name of the marker, after performs filtering on the sample, deleting impossible frames,
    performing median filter followed by MAD (median absolute deviation) filter.
    '''

    sample_size  = 10

    #retries per locate call
    max_retries = 3

    # Treshold values for median filter
    median_yaw_treshold = 0.05
    median_x_treshold   = 0.2
    median_y_treshold   = 0.2
    median_z_treshold   = 0.2

    #For catching frames that do not locate Nao on the ground
    ground_treshold = 0.4

    #For catching frames that do not locate Nao's feet parallel to the ground (in radians)
    ground_roll_treshold  = 0.2
    ground_pitch_treshold = 0.2 

    #Criteria for deciding on the marker choice
    max_left_marker_range = 1


    buffer_lock = None
    frame_buffer = None

    # True if buffer is latched and does not accept more frames
    buffer_latched = True

    def __init__(self, sample_size):
        self.sample_size = sample_size
        self.frames_buffer = defaultdict(list)
        self.buffer_lock = RLock()

    #Add frames to current dictionary (merge)
    def add_batch(self, frames_batch):
        self.buffer_lock.acquire(blocking=True)
        if not self.buffer_latched:
            self.frames_buffer = util.combine_dicts(self.frames_buffer, frames_batch)

        self.buffer_lock.release()


    def locate(self):
        '''
        Tries to locate Nao, until max_retries was excedeed
        Clears the buffer and waits until one marker fills the sample
        Applies the filtering and returns teh localized pose
        '''
        attemps = 1
        located = False
        localized_pose = None

        while not located:
            if attemps > self.max_retries:
                break
            self.clear_buffer()
            self.wait_for_buffer()
            localized_pose = self.apply_filter()
            if localized_pose != None:
                located = True    
            attemps += 1

        return localized_pose

    def wait_for_buffer(self):
        '''
        Waits until buffer is full
        '''
        r = rospy.Rate(10) # 10Hz
        self.buffer_latched = False
        while not self.is_buffer_ready():
            r.sleep()   
        self.buffer_latched = True            

    def is_buffer_ready(self):
        '''
        Determines if buffer is ready,
        by checking whether one of the marker is filled until @sample_size
        '''
        self.buffer_lock.acquire(blocking=True)
        ready = False
        for current_marker in self.frames_buffer.values():
            if len(current_marker) >= self.sample_size:
                ready = True
        
        self.buffer_lock.release()
        return ready

    def clear_buffer(self):
        '''
        Clears the buffer 
        '''
        self.buffer_lock.acquire(blocking=True)

        for k,v in self.frames_buffer.items():
            rospy.loginfo("Removing buffer: %s", k)
            del v[:]

        self.buffer_lock.release()

    def apply_filter(self):
        '''
        Applies filtering, first dropping impossible frames, after 
        applying median and MAD filter, and at the end deciding on the
        winning marker
        '''
        self.buffer_lock.acquire(blocking=True)
        rospy.loginfo("Applying filter...")
        
        marker_stats = defaultdict(SampleStat)

        for marker_name, cur_frames in self.frames_buffer.items():

            initial_frames_len = len(cur_frames)

            rospy.loginfo("  Processing marker: %s Frames caught: %d",
                marker_name, initial_frames_len)
            
            impossible_dropped  = self.drop_impossible_frames(marker_name)
            over_median_dropped = self.median_filter(marker_name)
            
            total_dropped = (impossible_dropped + over_median_dropped)
            mad = self.mad_filter(marker_name)

            marker_stats[marker_name] = SampleStat(dropped=total_dropped,
                                                   initial=initial_frames_len,
                                                   mad=mad)

        self.buffer_lock.release()
        rospy.loginfo("Finished filtering...")
        return self.decide_marker(marker_stats)

    def decide_marker(self, marker_stats):
        '''
        Decides the winning marker, by looking how many frames where dropped
        and choosing those which are at @max_left_marker_range far away from each other.
        After the winning marker is determined by getting lowest MAD from those markers.
        '''
        rospy.loginfo("Deciding marker...")

        rospy.loginfo("  {0:7} {1:7} {2:7}".format("Frames:", "dropped", "caught"))
        for k,v in marker_stats.items():    
            rospy.loginfo("  {0:7} {1:7d} {2:7d}".format(k, v.dropped, v.initial))

        max_left_frames_id, max_left_frames = max(self.frames_buffer.items(), key=lambda x: len(x[1]))

        if not max_left_frames:
            rospy.loginfo("  No valid markers found")
            return
        else:
            rospy.loginfo("  Marker with max no. frames left: %s", max_left_frames_id)

        markers_matching_max = [k for k,v in self.frames_buffer.items()
                                 if abs(len(v) - len(max_left_frames)) <= self.max_left_marker_range]

        rospy.loginfo("    Markers qualified for best mad ")
        rospy.loginfo("      {0:7} {1:7}".format("Marker:", "mad"))
        for marker_name in markers_matching_max:
            rospy.loginfo("      {0:7} {1:.3f}".format(marker_name, marker_stats[marker_name].mad))

        best_marker_name = min(markers_matching_max, key=lambda x: marker_stats[x].mad)

        rospy.loginfo("    Decision phase finished.")
        rospy.loginfo("    Best marker is: %s", best_marker_name)

        best_frames = self.frames_buffer[best_marker_name]

        average_frames    = util.avg_transforms(best_frames)
        random_odom_frame = random.choice(best_frames)[2]

        return (best_marker_name, average_frames, random_odom_frame)

    def mad_filter(self, marker_name):
        '''
        MAD filter calculated MAD for each marker,
        and drops all which have the distance to the average higher than MAD
        '''
        cur_frames = self.frames_buffer[marker_name]

        if len(cur_frames) == 0:
            return

        if len(cur_frames) == 2:
            del cur_frames[random.randint(0,1)] 
            return

        rospy.loginfo("  Applying distance filter")

        translations = [x[0] for x in cur_frames]
        yaws = [f[1][2] for f in cur_frames]

        avg_point = util.apply_func(translations, util.avg_alist)
        avg_yaw = util.avg_alist(yaws)
        rospy.loginfo("  {0} Averages:".format(marker_name))
        rospy.loginfo("    yaw: {0:.3f} x: {1:.3f} y: {2:.3f} z: {3:.3f}".format(avg_yaw, *avg_point))
        distances = [util.distance(avg_point, point) for point in translations]     
        mad = util.avg_alist(distances)

        cur_frames[:] = [llist[0:3] + [distance] for llist, distance in zip(cur_frames, distances)]
        cur_frames[:] = [f for f in cur_frames if f[3] < mad]

        print "DEBUG"
        for mar in self.frames_buffer[marker_name]:
            print mar[0], mar[3]
        print "DEBUG"

        return mad

    def median_filter(self, marker_name):
        '''
        Median filter drops all that are above the median tresholds
        '''
        cur_frames = self.frames_buffer[marker_name]

        if len(cur_frames) == 0:
            return 0
        
        rospy.loginfo("  Applying median filter")

        translations = [x[0] for x in cur_frames]
        yaws = [f[1][2] for f in cur_frames]

        median_point = util.apply_func(translations, util.median_alist)
        median_yaw = util.median_alist(yaws)
        medians = list(median_point) + [median_yaw]
        rospy.loginfo("  {0} Medians:".format(marker_name))
        rospy.loginfo("    x: {0:.3f} y: {1:.3f} z: {2:.3f} yaw: {3:.3f}".format(*medians))

        before_frames_len = len(cur_frames)

        cur_frames[:] = [f for f in cur_frames if self.determine_median_validity(f, medians)]

        return before_frames_len - len(cur_frames)

    def drop_impossible_frames(self, marker_name):
        '''
        Drop impossible filter, drops all frames which are way above/beneath the ground (look tresholds)
        or the orientation of the robot makes it impossible for him to move (e.g. it's not standing)
        '''
        rospy.loginfo("  Dropping impossible frames")
        
        cur_frames = self.frames_buffer[marker_name]
        before_frames_len = len(cur_frames)

        cur_frames[:] = [f for f in cur_frames if self.determine_map_validity(f)]

        return before_frames_len - len(cur_frames)

    def determine_median_validity(self, frame, medians):
        '''
        Determine function which compares medians and frames
        '''
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
        '''
        Determine function which compares frames to impossible pose tresholds
        '''
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

    marker_names = [8,11,28,48,89]
    #marker_names = [28]

    map_frame_id    = "map"
    odom_frame_id   = "odom"
    camera_frame_id = "CameraTop_frame"
    base_footprint_frame_id = "base_footprint"

    tf_broadcaster = None
    tf_listener    = None
    marker_static_tranformations = defaultdict(tfmath.identity_matrix) 

    localize_srv_server = None

    pose_filter = None

    current_drift_correction = tfmath.identity_matrix()

    def __init__(self):
        self.pose_filter = PoseFilter(10)

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener    = tf.TransformListener()
        self.marker_names   = ["4x4_" + str(marker_name) for marker_name in self.marker_names]

        self.localize_srv_server = rospy.Service("localize_nao", Empty, self.handle_localization)

    def start(self):
        '''
        Starts drift correction, publishing current correction every 100 miliseconds
        '''
        self.get_static_marker_transforms()
        rospy.Subscriber("ar_pose_marker", ARMarkers, self.marker_callback, queue_size=1)
        r = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():     
            self.tf_broadcaster.sendTransform(self.current_drift_correction[0],
                                              self.current_drift_correction[1],
                                              rospy.Time.now(),
                                              self.odom_frame_id,
                                              self.map_frame_id)
            r.sleep()

    def get_static_marker_transforms(self):
        '''
        Gets static transformations from map frame to marker frame
        '''
        for marker_name in self.marker_names:
            rospy.loginfo("Loading static transform for marker %s", marker_name)
            self.tf_listener.waitForTransform(self.map_frame_id, marker_name, rospy.Time(), rospy.Duration(4.0))
            tran, rot = self.tf_listener.lookupTransform(self.map_frame_id, marker_name, rospy.Time(0))
            rospy.loginfo("Static TF: {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *tran))
            self.marker_static_tranformations[marker_name] = util.matrix_from_pose(tran, rot)

    def get_transform(self, from_frame, to_frame):
        '''
        Queries TF to get earliest transformation @from_frame to @to_frame
        '''
    	trans, rot = (None,None)
        self.tf_listener.waitForTransform(from_frame, to_frame, rospy.Time(), rospy.Duration(4.0))
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(from_frame, to_frame, now, rospy.Duration(4.0))
            trans, rot = self.tf_listener.lookupTransform(from_frame, to_frame, now)
        except (tf.Exception) as e:
            rospy.logwarn("Error while getting runtime transform \n %s", e)
        return util.matrix_from_pose(trans, rot)

    def update_drift(self, location_mat, odom_mat):
        '''
        Updates the current drift correction
        from map to base_footprint transformation and
        odom to base_footprint transformations
        '''
        tran, eulers = location_mat
        location_mat = util.matrix_from_pose(tran, tfmath.quaternion_from_euler(*eulers))
        inv_odom_mat = tfmath.inverse_matrix(odom_mat)
        drift_correction_mat = np.dot(location_mat, inv_odom_mat)
        self.current_drift_correction = util.pose_from_matrix(drift_correction_mat)

    def marker_callback(self, data):
        '''
        Receives raw data from ar_pose package
        and stores it to pose_filter
        '''
        if len(data.markers) < 1:
            #rospy.loginfo("No markers detected")
            return

        frames_batch = {}

        for marker in data.markers:

            marker_name = marker.header.frame_id

            if marker_name not in self.marker_static_tranformations.keys():
                continue

            marker_tf        = util.matrix_from_pose_msg(marker.pose.pose)
            inv_marker_tf    = tfmath.inverse_matrix(marker_tf)
            map_to_camera_tf = np.dot(self.marker_static_tranformations[marker_name], inv_marker_tf)
            
            camera_to_footprint_tf = self.get_transform(self.camera_frame_id, self.base_footprint_frame_id)
            odom_to_footprint_tf   = self.get_transform(self.odom_frame_id, self.base_footprint_frame_id)

            map_to_footprint_tf = np.dot(map_to_camera_tf, camera_to_footprint_tf)
            
            map_to_footprint_translation = tfmath.translation_from_matrix(map_to_footprint_tf)
            map_to_footprint_rotation    = tfmath.euler_from_matrix(map_to_footprint_tf)

            #rospy.loginfo("Foot Tran {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *map_to_footprint_translation))
            #rospy.loginfo("Foot Rot  {0} {1:.3f} {2:.3f} {3:.3f}".format(marker_name, *map_to_footprint_rotation))

            frames_batch[marker_name] = [[map_to_footprint_translation, map_to_footprint_rotation, odom_to_footprint_tf]]

        self.pose_filter.add_batch(frames_batch)

    def handle_localization(self, request):
        '''
        Handles localization service request
        '''
        rospy.loginfo("Received localization service request")
        result = self.pose_filter.locate()
        if not result:
            rospy.logwarn("Could not locate Nao.")
            return

        marker_name, location, odom = result
        rospy.loginfo("Located from %s at:", marker_name)
        rospy.loginfo("YAW: {0:.3f} POS: [{1:.3f},{2:.3f},{3:.3f}]".format(location[1][2], *location[0]))
        self.update_drift(location, odom)

if __name__ == '__main__':

    rospy.init_node('drift_correction')
    dc = DriftCorrection()
    dc.start()

