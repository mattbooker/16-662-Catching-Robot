import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight
from geometry_msgs.msg import PointStamped
import time

import rospy

MOVE_THRESHOLD = 0.02
ball_position = PointStamped()
prev_position = PointStamped()

def distance(a, b):
    a = a.point
    b = b.point
    dist = np.power(a.x - b.x, 2) + np.power(a.y - b.y, 2) + np.power(a.z - b.z, 2)
    
    return np.power(dist, 1/2)


def clamp(a, low, high):
    return max(min(a, high), low)

def ball_callback(data):
    ball_position.point.x = 0.5
    ball_position.point.y = clamp(data.point.y, -0.3, 0)
    ball_position.point.z = clamp(data.point.z, 0.2, 0.7)

def get_trajectory(target_msg, ts, T):
    target = target_msg.point
    cur_pose = fa.get_pose()
    target_pose = RigidTransform(translation=np.array([0.4, target.y, target.z]), rotation=cur_pose.rotation)

    weights = [min_jerk_weight(t, T) for t in ts]
    pose_traj = [cur_pose.interpolate_with(target_pose, w) for w in weights]
    z_stiffness_traj = [min_jerk(100, 800, t, T) for t in ts]

    return pose_traj, z_stiffness_traj

def getTimeSince(start_time):
    return rospy.Time.now().to_time() - start_time

from collections import defaultdict

if __name__ == "__main__":
    T = 3

    # Reset Frankarm joints
    fa = FrankaArm(with_gripper=False)
    # fa.reset_joints()

    # Go to starting position (needs to be done in joint space)
    fa.goto_joints([1.06118167, -1.47672282, -1.71470848, -2.25366571,  0.02716874, 1.74172283, -0.95480864], duration=T, ignore_virtual_walls=True, use_impedance=True)
    start_pose = fa.get_pose()
    j0 = fa.get_joints()

    T = RigidTransform(
                translation=np.array([0.4, -0.25, 0.3]),
                rotation=start_pose.rotation, 
                                    from_frame=start_pose.from_frame, to_frame="world")

    table = defaultdict(dict)
    count = 0

    '''
    Limits:
    x = -0.1 - 0.25
    y = -0.05 - 0.6+ (0.6 is way more than we need)
    '''
    xs = np.arange(-0.1, 0.25 + 0.01, 0.05) + start_pose.translation[0]
    ys = np.arange(-0.05, 0.4 + 0.01, 0.05) + start_pose.translation[1]

    print(start_pose.translation)

    for x in xs:
        for y in ys:

            T = RigidTransform(
                translation=np.array([x, y, start_pose.translation[-1]]),
                rotation=start_pose.rotation, 
                                    from_frame=start_pose.from_frame, to_frame="world")

            fa.goto_pose(T, ignore_virtual_walls=True)
            joints = fa.get_joints()

        
            saved_x = np.round(0.05 * np.round(x / 0.05), 2)
            saved_y = np.round(0.05 * np.round(y / 0.05), 2)
            print(saved_x, saved_y)
            table[saved_x][saved_y] = joints
    
    import pickle
    pickle.dump(table, open("lookup_table_5cm.p", 'wb'))

    # s = time.time()
    # speedy = np.vectorize(min_jerk)
    # joints_traj = speedy(100, 800, ts, T)
    # print(time.time() - s)
