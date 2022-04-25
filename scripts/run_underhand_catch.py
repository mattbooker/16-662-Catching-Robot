import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import JointPositionSensorMessage, ShouldTerminateSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight
from geometry_msgs.msg import PointStamped
import time

import pickle
import rospy

MOVE_THRESHOLD = 0.02
ball_position = PointStamped()
prev_position = PointStamped()

lookup_table = pickle.load(open("lookup_table_5cm.p", "rb"))
# max_x = max(lookup_table.keys())
max_x = 0.55
min_x = min(lookup_table.keys())
# max_y = max(lookup_table[max_x].keys())
max_y = 0.1
min_y = min(lookup_table[min_x].keys())

def distance(a, b):
    a = a.point
    b = b.point
    dist = np.power(a.x - b.x, 2) + np.power(a.y - b.y, 2) + np.power(a.z - b.z, 2)
    
    return np.power(dist, 1/2)

def clamp(a, low, high):
    return max(min(a, high), low)

def ball_callback(data):
    ball_position.point.x = clamp(data.point.x, min_x, max_x)
    ball_position.point.y = clamp(data.point.y, min_y, max_y)
    ball_position.point.z = data.point.z

def get_joints(target_msg, ts, T):
    position = target_msg.point
    x = np.round(0.05 * np.round(position.x / 0.05), 2)
    y = np.round(0.05 * np.round(position.y / 0.05), 2)

    x = clamp(x, min_x, max_x)
    y = clamp(y, min_y, max_y)

    j0 = fa.get_joints()
    j1 = lookup_table[x][y]

    joints_traj = [min_jerk(j0, j1, t, T) for t in ts]
    
    return joints_traj

def getTimeSince(start_time):
    return rospy.Time.now().to_time() - start_time

if __name__ == "__main__":
    T = 0.5
    dt = 0.02
    ts = np.arange(0, T, dt)
    buffer_time = 10

    traj_idx = 0
    msg_id = 0

    # Reset Frankarm joints
    fa = FrankaArm(with_gripper=False)
    # fa.reset_joints()

    # Setup pub and sub
    rospy.loginfo('Initializing Publishers')
    arm_pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    ball_sub = rospy.Subscriber("catch_point", PointStamped, ball_callback)
    rate = rospy.Rate(1 / dt)

    # Go to starting position (needs to be done in joint space)
    fa.goto_joints([1.06118167, -1.47672282, -1.71470848, -2.25366571,  0.02716874, 1.74172283, -0.95480864], duration=5, ignore_virtual_walls=True, use_impedance=True)
    p0 = fa.get_pose()
    j0 = fa.get_joints()

    '''
    Limits:
    x = -0.1 - 0.25
    y = -0.05 - 0.6+ (0.6 is way more than we need)
    '''
    # T_delta = RigidTransform(
    #     translation=np.array([0, 0, 0]),
    #     rotation=RigidTransform.x_axis_rotation(np.deg2rad(0)), 
    #                         from_frame="world", to_frame=p0.to_frame)

    # p1 = T_delta * p0

    # fa.goto_pose(p1, ignore_virtual_walls=True)
    # j1 = fa.get_joints()

    rospy.loginfo('Starting control loop')
    joints_traj = [min_jerk(j0, j0, t, T) for t in ts]

    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_joints(joints_traj[1], duration=T, dynamic=True, buffer_time=buffer_time)

    init_time = rospy.Time.now().to_time()

    while getTimeSince(init_time) < buffer_time - 2:

        # rospy.loginfo(f'{ball_position}, {prev_position}')

        if distance(ball_position, prev_position) > MOVE_THRESHOLD:
            prev_position.point.x = ball_position.point.x
            prev_position.point.y = ball_position.point.y
            prev_position.point.z = ball_position.point.z

            # Get new trajectory and reset index
            print("NEW PLAN")
            print("--------")

            joints_traj = get_joints(ball_position, ts, T)
            traj_idx = 0

        # Prevent out of bounds error
        traj_idx = min(traj_idx, len(joints_traj) - 1)

        timestamp = rospy.Time.now().to_time() - init_time

        traj_gen_proto_msg = JointPositionSensorMessage(
            id=msg_id, timestamp=rospy.Time.now().to_time() - init_time, 
            joints=joints_traj[traj_idx]
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.JOINT_POSITION)
        )

        traj_idx += 1
        msg_id += 1
        
        # rospy.loginfo(f'Current Pose: {fa.get_pose().translation}')
        # rospy.loginfo(f'Target: {ball_position.point.y}, {ball_position.point.z}')
        # rospy.loginfo(f'---------------')
        arm_pub.publish(ros_msg)
        rate.sleep()

    # Stop the skill
    # Alternatively can call fa.stop_skill()
    term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    ros_msg = make_sensor_group_msg(
        termination_handler_sensor_msg=sensor_proto2ros_msg(
            term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        )
    arm_pub.publish(ros_msg)
    
    rospy.loginfo('Done')
