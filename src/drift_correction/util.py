#!/usr/bin/env python
import tf.transformations as tfmath
import numpy as np
import math, operator, random

from geometry_msgs.msg import (Point, Quaternion)

def matrix_from_pose(tran, rot):
    translation_matrix = tfmath.translation_matrix(tran)
    quaternion_matrix  = tfmath.quaternion_matrix(rot)
    return np.dot(translation_matrix, quaternion_matrix)

def matrix_from_pose_msg(pose):
    translation = tuple_from_vector(pose.position)
    orientation = tuple_from_quat(pose.orientation)
    return matrix_from_pose(translation, orientation)

def pose_from_matrix(mat):
    tran = tfmath.translation_from_matrix(mat)
    quat = tfmath.quaternion_from_matrix(mat)
    return (tran, quat)

def pose_msg_from_matrix(mat):
    pose = pose_from_matrix(mat)
    return pose_msg_from_pose(pose)

def pose_msg_from_pose(pose):
    return (Point(*pose[0]), Quaternion(*pose[1]))

def tran_and_euler_from_matrix(mat):
    pose = pose_from_matrix(mat)
    return (pose[0], tfmath.euler_from_quaternion(pose[1]))

def avg_transforms(transforms):
    translations = [transform[0] for transform in transforms]
    yaws         = [transform[1][2] for transform in transforms]

    avg_tran = apply_func(translations, avg_alist)
    avg_yaw  = avg_alist(yaws)
    avg_rot  = (0,0,avg_yaw)

    return (avg_tran, avg_rot)

def tuple_from_vector(vector):
    return (vector.x, vector.y, vector.z)

def tuple_from_quat(quat):
    return (quat.x, quat.y, quat.z, quat.w)

def length(vec):
    return distance((0,0,0), vec)

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2 + (p0[2] - p1[2])**2)

def median_alist(alist):
    sorted_list = sorted(alist)
    length = len(sorted_list)
    if not length % 2:
        return (sorted_list[length/2]+sorted_list[length/2-1])/2.0
    else:
        return (sorted_list[length/2])  

def avg_alist(alist):
    return reduce(lambda x, y: x + y, alist) / len(alist)

def apply_func(translations, func):
    xs, ys, zs = zip(*translations)
    return func(xs), func(ys), func(zs)

def combine_dicts(a, b, op=operator.add):
    return dict(a.items() + b.items() +
        [(k, op(a[k], b[k])) for k in b.viewkeys() & a.viewkeys()])