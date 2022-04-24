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
    # ball_position.point.x = 0.35
    # ball_position.point.y = max(min(data.point.y, 0), -0.3)
    # ball_position.point.z = max(min(data.point.z, 0.7), 0.2)

    ball_position.point.x = 0.4
    ball_position.point.y = clamp(data.point.y, -0.3, 0)
    ball_position.point.z = clamp(data.point.z, 0.2, 0.7)

def get_trajectory(target_msg, ts, T):
    target = target_msg.point
    cur_pose = fa.get_pose()
    # target_pose = RigidTransform(translation=np.array([0.35, target.y, target.z]), rotation=cur_pose.rotation)
    target_pose = RigidTransform(translation=np.array([0.4, target.y, target.z]), rotation=cur_pose.rotation)

    weights = [min_jerk_weight(t, T) for t in ts]
    pose_traj = [cur_pose.interpolate_with(target_pose, w) for w in weights]
    z_stiffness_traj = [min_jerk(100, 800, t, T) for t in ts]

    return pose_traj, z_stiffness_traj

def getTimeSince(start_time):
    return rospy.Time.now().to_time() - start_time

if __name__ == "__main__":
    T = 2
    dt = 0.02
    ts = np.arange(0, T, dt)
    buffer_time = 30

    traj_idx = 0
    msg_id = 0

    # Reset Frankarm joints
    fa = FrankaArm()
    fa.reset_joints()

    # Setup pub and sub
    rospy.loginfo('Initializing Publishers')
    arm_pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    ball_sub = rospy.Subscriber("ball_pose", PointStamped, ball_callback)
    rate = rospy.Rate(1 / dt)

    # Create starting position
    p0 = fa.get_pose()
    p1 = p0.copy()
    T_delta = RigidTransform(
        translation=np.array([0, 0, 0]),
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
    p1 = p1 * T_delta

    # Create trajectory to starting pose
    weights = [min_jerk_weight(t, T) for t in ts]
    pose_traj = [p0.interpolate_with(p1, w) for w in weights]
    z_stiffness_traj = [min_jerk(100, 800, t, T) for t in ts]

    rospy.loginfo('Starting control loop')
    # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    fa.goto_pose(pose_traj[1], duration=T, dynamic=True, buffer_time=buffer_time)

    init_time = rospy.Time.now().to_time()

    while getTimeSince(init_time) < buffer_time - 1:

        # rospy.loginfo(f'{ball_position}, {prev_position}')

        if distance(ball_position, prev_position) > MOVE_THRESHOLD:
            prev_position.point.x = ball_position.point.x
            prev_position.point.y = ball_position.point.y
            prev_position.point.z = ball_position.point.z

            # Get new trajectory and reset index
            print("NEW PLAN")
            print("--------")

            pose_traj, z_stiffness_traj = get_trajectory(ball_position, ts, T)
            traj_idx = int(len([pose_traj]) / 3)

            print(len(pose_traj))

        # Prevent out of bounds error
        traj_idx = min(traj_idx, len(pose_traj) - 1)

        timestamp = rospy.Time.now().to_time() - init_time

        traj_gen_proto_msg = PosePositionSensorMessage(
            id=msg_id, timestamp=timestamp, 
            position=pose_traj[traj_idx].translation, quaternion=pose_traj[traj_idx].quaternion
        )
        fb_ctrlr_proto = CartesianImpedanceSensorMessage(
            id=msg_id, timestamp=timestamp,
            translational_stiffnesses=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES[:2] + [z_stiffness_traj[traj_idx]],
            rotational_stiffnesses=FC.DEFAULT_ROTATIONAL_STIFFNESSES
        )
        ros_msg = make_sensor_group_msg(
            trajectory_generator_sensor_msg=sensor_proto2ros_msg(
                traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
            feedback_controller_sensor_msg=sensor_proto2ros_msg(
                fb_ctrlr_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)
        )

        traj_idx += 1
        msg_id += 1
        
        rospy.loginfo(f'Current Pose: {fa.get_pose().translation}')
        rospy.loginfo(f'Target: {ball_position.point.y}, {ball_position.point.z}')
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
