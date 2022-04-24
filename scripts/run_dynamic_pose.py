import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight

import rospy


if __name__ == "__main__":
    fa = FrankaArm()

    fa.reset_joints()

    rospy.loginfo('Generating Trajectory')
    p0 = fa.get_pose()
    p1 = RigidTransform(
        translation=np.array([0.40832528, -0.0464719, 0.1433692]), 
        rotation=np.array([[ 0.70225965, 0.01181182, 0.71181624],
                            [0.00149043, -0.99987492,  0.01512171],
                            [0.71190583, -0.00955845, -0.70220294]]), 
                            from_frame="franka_tool", to_frame="world")
    fa.goto_pose(p1, duration=4)

    # T = 2
    # dt = 0.02
    # ts = np.arange(0, T, dt)
    # current_gripper_width = 0.08
    # has_closed = False

    # weights = [min_jerk_weight(t, T) for t in ts]
    # pose_traj = [p1.interpolate_with(p0, w) for w in weights]

    # z_stiffness_traj = [min_jerk(100, 800, t, T) for t in ts]

    # rospy.loginfo('Initializing Sensor Publisher')
    # pub = rospy.Publisher(FC.DEFAULT_SENSOR_PUBLISHER_TOPIC, SensorDataGroup, queue_size=1000)
    # rate = rospy.Rate(1 / dt)

    # rospy.loginfo('Publishing pose trajectory...')

    
    # # To ensure skill doesn't end before completing trajectory, make the buffer time much longer than needed
    # fa.goto_pose(pose_traj[1], duration=T, dynamic=True, buffer_time=10,
    #     cartesian_impedances=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES[:2] + [z_stiffness_traj[1]] + FC.DEFAULT_ROTATIONAL_STIFFNESSES
    # )
    # init_time = rospy.Time.now().to_time()

    # for i in range(2, len(ts)):
    #     timestamp = rospy.Time.now().to_time() - init_time
    #     traj_gen_proto_msg = PosePositionSensorMessage(
    #         id=i, timestamp=timestamp, 
    #         position=pose_traj[i].translation, quaternion=pose_traj[i].quaternion
    #     )
    #     fb_ctrlr_proto = CartesianImpedanceSensorMessage(
    #         id=i, timestamp=timestamp,
    #         translational_stiffnesses=FC.DEFAULT_TRANSLATIONAL_STIFFNESSES[:2] + [z_stiffness_traj[i]],
    #         rotational_stiffnesses=FC.DEFAULT_ROTATIONAL_STIFFNESSES
    #     )
    #     ros_msg = make_sensor_group_msg(
    #         trajectory_generator_sensor_msg=sensor_proto2ros_msg(
    #             traj_gen_proto_msg, SensorDataMessageType.POSE_POSITION),
    #         feedback_controller_sensor_msg=sensor_proto2ros_msg(
    #             fb_ctrlr_proto, SensorDataMessageType.CARTESIAN_IMPEDANCE)
    #         )

    #     if not has_closed:
    #         current_gripper_width -= 0.0005
    #     else:
    #         current_gripper_width += 0.0005

    #     if current_gripper_width < 0.002:
    #         has_closed = True

    #     fa.goto_gripper(current_gripper_width, block=False)

        
    #     rospy.loginfo('Publishing: ID {}'.format(traj_gen_proto_msg.id))
    #     rospy.loginfo(f'{timestamp}')
    #     pub.publish(ros_msg)
    #     rate.sleep()

    # # Stop the skill
    # # Alternatively can call fa.stop_skill()
    # term_proto_msg = ShouldTerminateSensorMessage(timestamp=rospy.Time.now().to_time() - init_time, should_terminate=True)
    # ros_msg = make_sensor_group_msg(
    #     termination_handler_sensor_msg=sensor_proto2ros_msg(
    #         term_proto_msg, SensorDataMessageType.SHOULD_TERMINATE)
    #     )
    # pub.publish(ros_msg)
    
    rospy.loginfo('Done')
