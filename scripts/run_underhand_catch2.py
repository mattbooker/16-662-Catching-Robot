from mimetypes import init
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

import pickle
import rospy

MOVE_THRESHOLD = 0.02
ball_position = PointStamped()
prev_position = PointStamped()

lookup_table = pickle.load(open("lookup_table_5cm.p", "rb"))
# max_x = max(lookup_table.keys())
max_x = 0.6
min_x = 0.35
# max_y = max(lookup_table[max_x].keys())
max_y = 0.35
min_y = -0.35

def distance(a, b):
    a = a.point
    b = b.point
    dist = np.power(a.x - b.x, 2) + np.power(a.y - b.y, 2) + np.power(a.z - b.z, 2)
    
    return np.power(dist, 1/2)

def clamp(a, low, high):
    return max(min(a, high), low)

def ball_callback(data):
    # Only react if message newer than a second
    if (rospy.Time.now() - data.header.stamp > rospy.Duration(1)):
        return

    # rospy.loginfo(f'Now = {rospy.Time.now()}, Sent = {data.header.stamp}, diff = {rospy.Time.now() - data.header.stamp}')


    ball_position.point.x = clamp(data.point.x, min_x, max_x)
    ball_position.point.y = clamp(data.point.y, min_y, max_y)
    ball_position.point.z = clamp(data.point.z, 0.3, 0.6)

def get_pose(target_msg, ts, T):
    position = target_msg.point


    x = clamp(position.x, min_x, max_x)
    y = clamp(position.y, min_y, max_y)
    z = clamp(position.z - 0.075, 0.3, 0.6)

    p0 = fa.get_pose()

    p1 = RigidTransform(
        translation=np.array([x, y, z]),
        rotation=p0.rotation, 
        from_frame=p0.from_frame, to_frame="world")

    weights = [min_jerk_weight(t, T) for t in ts]
    pose_traj = [p0.interpolate_with(p1, w) for w in weights]

    # rospy.loginfo(f'Finished plan at {rospy.Time.now()}')
    
    return pose_traj

def getTimeSince(start_time):
    return rospy.Time.now().to_time() - start_time

if __name__ == "__main__":
    T = 0.5
    dt = 0.01
    ts = np.arange(0, T, dt)
    buffer_time = 100

    traj_idx = 0
    msg_id = 0

    # Reset Frankarm joints
    fa = FrankaArm(with_gripper=False)

    # Setup pub and sub
    rospy.loginfo('Initializing Publishers')
    arm_pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    ball_sub = rospy.Subscriber("catch_point", PointStamped, ball_callback)
    rate = rospy.Rate(1 / dt)

    while True:
        fa.reset_joints()
        start_pose = fa.get_pose()
        # Go to starting position (needs to be done in joint space)
        fa.goto_pose(RigidTransform(
            translation=np.array([0.5, -0.2, 0.4]),
            rotation=start_pose.rotation, 
            from_frame=start_pose.from_frame, to_frame="world"), duration=5, ignore_virtual_walls=True, use_impedance=True)
        p0 = fa.get_pose()

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
        weights = [min_jerk_weight(t, T) for t in ts]
        pose_traj = [p0.interpolate_with(p0, w) for w in weights]
        # z_stiffness_traj = [min_jerk(100, 800, t, T) for t in ts]

        # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
        fa.goto_pose(pose_traj[1], duration=T, dynamic=True, buffer_time=buffer_time)

        init_time = rospy.Time.now().to_time()
        start_time = rospy.Time.now()

        while getTimeSince(init_time) < buffer_time - 1:

            # rospy.loginfo(f'{ball_position.point}, {prev_position.point}')

            if rospy.Time.now() - start_time > rospy.Duration(6):
                break

            if distance(ball_position, prev_position) > MOVE_THRESHOLD:
                prev_position.point.x = ball_position.point.x
                prev_position.point.y = ball_position.point.y
                prev_position.point.z = ball_position.point.z

                # Get new trajectory and reset index
                print("NEW PLAN")
                print("--------")

                pose_traj = get_pose(ball_position, ts, T)
                traj_idx = int(len(pose_traj) / 3)

            # Prevent out of bounds error
            traj_idx = min(traj_idx, len(pose_traj) - 1)

            timestamp = rospy.Time.now().to_time() - init_time

            traj_gen_proto_msg = PosePositionSensorMessage(
                id=msg_id, timestamp=timestamp, 
                position=pose_traj[traj_idx].translation, quaternion=pose_traj[traj_idx].quaternion
            )
            ros_msg = make_sensor_group_msg(
                trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                    traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
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
        # term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
        # ros_msg = make_sensor_group_msg(
        #     termination_handler_sensor_msg=sensor_proto2ros_msg(
        #         term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
        #     )
        # arm_pub.publish(ros_msg)
        fa.stop_skill()
        
        rospy.loginfo('Done')
