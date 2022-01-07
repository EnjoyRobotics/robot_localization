#! /usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Transform, Pose, Vector3, Quaternion
from scipy.spatial.transform import Rotation
from conversion import convert


def to_homogenius(mat: np.ndarray) -> np.ndarray:
    if mat.ndim == 1: # vector
        new = np.ones((mat.shape[0]+1,), dtype=mat.dtype)
        new[0:-1] = mat
    elif mat.ndim == 2: # matrix
        new = np.zeros((mat.shape[0]+1, mat.shape[1]+1), dtype=mat.dtype)
        new[:-1, :-1] = mat
        new[-1, -1] = 1
    else: # ???
        raise NotImplementedError
    return new


def Transform_to_T(tf: Transform, *, invert: bool=False) -> np.ndarray:
    T_trans = convert(tf.translation, np.ndarray)
    if invert:
        T_trans = -T_trans

    q = convert(tf.rotation, np.ndarray)
    T_rot = Rotation.from_quat(q).as_matrix()
    if invert:
        T_rot = np.linalg.inv(T_rot)

    T = to_homogenius(T_rot)
    T[:3, -1] = T_trans

    return T


def T_to_Transform(T: np.ndarray) -> Transform:
    tf = Transform()

    T_trans = T[:3, -1]
    tf.translation = convert(T_trans, Vector3)

    T_rot   = T[:3, :3]
    q = Rotation.from_matrix(T_rot).as_quat()
    tf.rotation = convert(q, Quaternion)

    return tf


def transform(pose: Pose, tf: Transform) -> Pose:
    tf_OC = convert(pose, Transform)
    T_OC  = Transform_to_T(tf_OC)
    tf_CB = tf
    T_CB  = Transform_to_T(tf_CB)

    T_OB  = T_OC.dot(T_CB)
    tf_OB = T_to_Transform(T_OB)
    return convert(tf_OB, Pose)


if __name__ == '__main__':
    tf = Transform()
    tf.translation.y=8.
    tf.rotation.x = .5
    Transform_to_T(tf)
