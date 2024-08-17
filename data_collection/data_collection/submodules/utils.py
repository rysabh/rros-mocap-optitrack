from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np
import pandas as pd

from scipy.spatial.transform import Rotation
from scipy.ndimage import generic_filter
from scipy.signal import find_peaks

from typing import List

def check_same_pose(pose_1: Pose, pose_2: Pose) -> bool:
    x_diff = np.abs(pose_1.position.x - pose_2.position.x)
    y_diff = np.abs(pose_1.position.y - pose_2.position.y)
    z_diff = np.abs(pose_1.position.z - pose_2.position.z)

    pos_diff = np.asarray([x_diff, y_diff, z_diff])

    qx_diff = np.abs(pose_1.orientation.x - pose_2.orientation.x)
    qy_diff = np.abs(pose_1.orientation.y - pose_2.orientation.y)
    qz_diff = np.abs(pose_1.orientation.z - pose_2.orientation.z)
    qw_diff = np.abs(pose_1.orientation.w - pose_2.orientation.w)

    quat_diff = np.asarray([qx_diff, qy_diff, qz_diff, qw_diff])

    return np.linalg.norm(pos_diff) < 0.001 and np.linalg.norm(quat_diff) < 0.001


def construct_transform_matrix(
    rotation: np.ndarray, translation: np.ndarray
) -> np.ndarray:
    translation = translation.reshape(3, 1)

    transform = np.identity(4)
    transform[0:3, 0:3] = rotation
    transform[0:3, [3]] = translation

    return transform


def construct_pose(transform: np.ndarray) -> Pose:
    rotation = transform[0:3, 0:3]
    translation = transform[0:3, 3].reshape(1, 3)

    # scipy quat defination: quat = [x, y, z, w]
    quat = Rotation.from_matrix(rotation).as_quat()

    return Pose(
        position=Point(
            x=translation[0][0],
            y=translation[0][1],
            z=translation[0][2],
        ),
        orientation=Quaternion(w=quat[3], x=quat[0], y=quat[1], z=quat[2]),
    )


def construct_offset_dict(waypoint_difference: np.ndarray) -> dict[str, float]:
    # euler = Rotation.from_quat(waypoint_difference[3:]).as_euler("ZYX", degrees=True)
    return {
        "X": waypoint_difference[0] / 1e3,
        "Y": waypoint_difference[1] / 1e3,
        "Z": waypoint_difference[2] / 1e3,
        "A": waypoint_difference[3],
        "B": waypoint_difference[4],
        "C": waypoint_difference[5],
    }

    
def get_peg_to_hole_transform(perfect_aligned_pose: Pose, flange_pose: Pose, sensor_transform: np.array) -> np.array:
    t_HB = np.linalg.inv(sensor_transform) @ construct_transform_matrix(
        rotation=Rotation.from_quat(np.array([perfect_aligned_pose.orientation.x, perfect_aligned_pose.orientation.y, perfect_aligned_pose.orientation.z, perfect_aligned_pose.orientation.w])).as_matrix(),
        translation=np.array([perfect_aligned_pose.position.x, perfect_aligned_pose.position.y, perfect_aligned_pose.position.z])
    )
    t_BF = construct_transform_matrix(
        rotation=Rotation.from_quat(np.array([flange_pose.orientation.x, flange_pose.orientation.y, flange_pose.orientation.z, flange_pose.orientation.w])).as_matrix(),
        translation=np.array([flange_pose.position.x, flange_pose.position.y, flange_pose.position.z])
    )
    t_FP = sensor_transform
    t_HP = t_HB @ t_BF @ t_FP

    return t_HP

def get_offset_pose(base_pose: Pose, offset_dict: dict[str, float]) -> Pose:
    offset_rotation = Rotation.from_euler(
        seq="XYZ",
        angles=[offset_dict["C"], offset_dict["B"], offset_dict["A"]],
        degrees=True,
    ).as_matrix()
    offset_translation = np.asarray(
        [offset_dict["X"], offset_dict["Y"], offset_dict["Z"]]
    ).reshape(3, 1)

    transform = construct_transform_matrix(offset_rotation, offset_translation)

    base_rotation = Rotation.from_quat(
        [
            base_pose.orientation.x,
            base_pose.orientation.y,
            base_pose.orientation.z,
            base_pose.orientation.w,
        ]
    ).as_matrix()
    base_translation = np.asarray(
        [base_pose.position.x, base_pose.position.y, base_pose.position.z]
    ).reshape(3, 1)

    base = construct_transform_matrix(base_rotation, base_translation)

    offset_transform = np.matmul(base, transform)

    return construct_pose(offset_transform)

def quanterion_to_euler(quat):
    
    x, y, z, w = quat.x , quat.y,quat.z,quat.w
    r = Rotation.from_quat([x,y,z,w])
    euler_angles = r.as_euler('xyz' , degrees = True)

    return euler_angles


def pose_to_position_and_euler(pose : Pose) -> dict:
    # Extract position and convert to millimeters
    position = pose.position
    position_mm = {
        "x_mm" : position.x * 1000,
        "y_mm" : position.y * 1000,
        "z_mm" : position.z * 1000
    }

    # Extract Orientation
    orientation = pose.orientation
    euler_angles = quanterion_to_euler(orientation)

    # create a dict for results
    result = {
        'position_mm' : position_mm,
        'euler_angles' : {
            'A' : euler_angles[0],
            'B' : euler_angles[1],
            'C' : euler_angles[2]
        }
    }

    return result


def frange(start, stop, step, n=None):
    """return a WYSIWYG series of float values that mimic range behavior
    by excluding the end point and not printing extraneous digits beyond
    the precision of the input numbers (controlled by n and automatically
    detected based on the string representation of the numbers passed).

    EXAMPLES
    ========

    non-WYSIWYS simple list-comprehension

    >>> [.11 + i*.1 for i in range(3)]
    [0.11, 0.21000000000000002, 0.31]

    WYSIWYG result for increasing sequence

    >>> list(frange(0.11, .33, .1))
    [0.11, 0.21, 0.31]

    and decreasing sequences

    >>> list(frange(.345, .1, -.1))
    [0.345, 0.245, 0.145]

    To hit the end point for a sequence that is divisibe by
    the step size, make the end point a little bigger by
    adding half the step size:

    >>> dx = .2
    >>> list(frange(0, 1 + dx/2, dx))
    [0.0, 0.2, 0.4, 0.6, 0.8, 1.0]

    """
    if step == 0:
        raise ValueError("step must not be 0")
    # how many decimal places are showing?
    if n is None:
        n = max(
            [
                0 if "." not in str(i) else len(str(i).split(".")[1])
                for i in (start, stop, step)
            ]
        )
    if step * (stop - start) > 0:  # a non-null incr/decr range
        if step < 0:
            for i in frange(-start, -stop, -step, n):
                yield -i
        else:
            steps = round((stop - start) / step)
            while round(step * steps + start, n) < stop:
                steps += 1
            for i in range(steps):
                yield round(start + i * step, n)


def flat_region_filter(data: np.array, window_size=200):
    stddev = generic_filter(data, np.std, size=window_size)
    movmean_stddev = pd.Series(stddev).rolling(window=window_size).mean()
    idx_min_std = np.argmin(movmean_stddev)
    idx_flat = np.arange(max([0, idx_min_std - window_size]), idx_min_std).astype(int)
    data_flat_filt = data[idx_flat]
    return idx_flat, data_flat_filt


def filter_wrench_flat(data: pd.DataFrame, window_size=200) -> np.array:
    FZ = data.loc[:, "FZ"].to_numpy()
    idx_flat, _ = flat_region_filter(FZ, window_size)
    wrench_filtered = data.loc[
        idx_flat, ["FX", "FY", "FZ", "TX", "TY", "TZ"]
    ].to_numpy()
    wrench_filtered_mean = np.mean(wrench_filtered, axis=0)
    return wrench_filtered_mean


def clip_past_contact(data: pd.DataFrame):
    FZ = data.loc[:, "FZ"].to_numpy()
    # idx_peaks, properties = find_peaks(np.diff(np.array(-data.loc[:,'FZ'])), distance=500)
    idx_peaks, properties = find_peaks(
        (np.array(-data.loc[:, "FZ"])), distance=500, height=20
    )
    (
        properties["peak_heights"]
        if "peak_heights" in properties
        else np.diff(np.array(-data.loc[idx_peaks, "FZ"]))
    )
    num_peaks = min([2, len(idx_peaks)])
    min_samples = 250

    if len(idx_peaks) != 0:
        idx_clip = idx_peaks[-num_peaks]
    else:
        idx_clip = len(data) - min_samples

    idx_clip = min(
        idx_clip, len(data) - min_samples
    )  # have at least min num of samples
    data_clipped = data.loc[idx_clip:, :]
    data_clipped = data_clipped.reset_index(drop=True)

    return data_clipped, idx_clip, idx_peaks

