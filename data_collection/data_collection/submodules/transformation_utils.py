import numpy as np 
from scipy.spatial.transform import Rotation as R 
from geometry_msgs.msg import Pose, Point, Quaternion

''' 
    pose: Pose(position, orientation) [meters, quaternion]
    transform: np.array((4,4)) [meters, rotation matrix]  
    xyzabc: np.array(6) [millimeters, degrees] 
    xyzabc_split: 6 floats [millimeters, degrees] 
'''

def transform_to_pose(T: np.array) -> Pose: 
    q = R.from_matrix(T[:3,:3]).as_quat() 
    pose = Pose(
        position=Point(x=T[0,3], y=T[1,3], z=T[2,3]),
        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) 
    )
    return pose 

def pose_to_transform(pose: Pose) -> np.array: 
    p = np.array([[pose.position.x],[pose.position.y],[pose.position.z]])  
    r = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_matrix()  
    T = np.vstack((np.hstack((r,p)), np.array([0,0,0,1]))) 
    return T 

def pose_to_xyzabc(pose: Pose, split=False):
    X = pose.position.x * 1e3
    Y = pose.position.y * 1e3
    Z = pose.position.z * 1e3 
    C,B,A = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]).as_euler('xyz',degrees=True) 
    if split: 
        return X,Y,Z,A,B,C 
    else: 
        return np.array([X,Y,Z,A,B,C])

def xyzabc_to_pose(xyzabc) -> Pose: 
    X = xyzabc[0] / 1e3
    Y = xyzabc[1] / 1e3
    Z = xyzabc[2] / 1e3
    A = xyzabc[3] 
    B = xyzabc[4]
    C = xyzabc[5] 
    pose = Pose(
            position=Point(x=X, y=Y, z=Z),
            orientation=Quaternion(R.from_euler('xyz', [C,B,A], degrees=True).as_quat() ) 
        )
    return pose 

def xyzabc_to_transform(xyzabc) -> np.array: 
    X = xyzabc[0] / 1e3 
    Y = xyzabc[1] / 1e3
    Z = xyzabc[2] / 1e3
    A = xyzabc[3]
    B = xyzabc[4]
    C = xyzabc[5] 
    p = np.array([[X],[Y],[Z]])  
    r = R.from_euler('xyz', [C,B,A], degrees=True).as_matrix()  
    T = np.vstack((np.hstack((r,p)), np.array([0,0,0,1]))) 
    return T 

def transform_to_xyzabc(T: np.array, split=False):
    pose = transform_to_pose(T) 
    return pose_to_xyzabc(pose, split=split) 

def inverse_transform(T): 
    R = T[0:3,0:3]
    p = T[0:3,3]
    RT = np.transpose(R) 
    invp = -RT @ p
    invT = np.array([[RT[0,0], RT[0,1], RT[0,2], invp[0]],
                     [RT[1,0], RT[1,1], RT[1,2], invp[1]],
                     [RT[2,0], RT[2,1], RT[2,2], invp[2]], 
                     [0.,0.,0.,1.]])
    return invT 