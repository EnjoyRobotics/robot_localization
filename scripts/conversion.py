import numpy as np
from geometry_msgs.msg import Transform, Pose, Vector3, Quaternion, Point
from scipy.spatial.transform import Rotation
from typing import Union


# conversions
def Vector3_to_ndarray(pt: Vector3) -> np.ndarray:
    return np.array([pt.x, pt.y, pt.z])

def Quaternion_to_ndarray(q: Quaternion) -> np.ndarray:
    return np.array([q.x, q.y, q.z, q.w])

def Pose_to_Transform(pose: Pose) -> Transform:
    tf = Transform()
    tf.translation.x = pose.position.x
    tf.translation.y = pose.position.y
    tf.translation.z = pose.position.z

    tf.rotation.x = pose.orientation.x
    tf.rotation.y = pose.orientation.y
    tf.rotation.z = pose.orientation.z
    tf.rotation.w = pose.orientation.w

    return tf


def convert(data_in: Union[Vector3, Point, Quaternion, Pose, np.ndarray, Transform],
        out_dtype: Union[Vector3, Point, Quaternion, Pose, np.ndarray, Transform]):
    if type(data_in) in [Point, Vector3]:
        if out_dtype == np.ndarray:
            return np.array([data_in.x, data_in.y, data_in.z])
        else:
            raise NotImplementedError

    if type(data_in) == Quaternion:
        if out_dtype == np.ndarray:
            return np.array([data_in.x, data_in.y, data_in.z, data_in.w])
        else:
            raise NotImplementedError

    if type(data_in) == Transform:
        if out_dtype == Pose:
            pose = Pose()
            pose.position.x = data_in.translation.x
            pose.position.y = data_in.translation.y
            pose.position.z = data_in.translation.z
            pose.orientation.x = data_in.rotation.x
            pose.orientation.y = data_in.rotation.y
            pose.orientation.z = data_in.rotation.z
            pose.orientation.w = data_in.rotation.w
            return pose
        else:
            raise NotImplementedError

    if type(data_in) == Pose:
        if out_dtype == Transform:
            tf = Transform()
            tf.translation.x = data_in.position.x
            tf.translation.y = data_in.position.y
            tf.translation.z = data_in.position.z
            tf.rotation.x = data_in.orientation.x
            tf.rotation.y = data_in.orientation.y
            tf.rotation.z = data_in.orientation.z
            tf.rotation.w = data_in.orientation.w
            return tf
        else:
            raise NotImplementedError

    if type(data_in) == np.ndarray:
        if out_dtype == Vector3:
            pt = Vector3()
            pt.x = data_in[0]
            pt.y = data_in[1]
            pt.z = data_in[2]
            return pt
        if out_dtype == Point:
            pt = Point()
            pt.x = data_in[0]
            pt.y = data_in[1]
            pt.z = data_in[2]
            return pt
        if out_dtype == Quaternion:
            q = Quaternion()
            q.x = data_in[0]
            q.y = data_in[1]
            q.z = data_in[2]
            q.w = data_in[3]
            return q
