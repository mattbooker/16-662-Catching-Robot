import numpy as np

from autolab_core import RigidTransform
from frankapy import FrankaArm, SensorDataMessageType
from frankapy import FrankaConstants as FC
from frankapy.proto_utils import sensor_proto2ros_msg, make_sensor_group_msg
from frankapy.proto import PosePositionSensorMessage, ShouldTerminateSensorMessage, CartesianImpedanceSensorMessage
from franka_interface_msgs.msg import SensorDataGroup

from frankapy.utils import min_jerk, min_jerk_weight
import time

import rospy


if __name__ == "__main__":
    fa = FrankaArm()
    fa.reset_joints()

    rospy.loginfo('Generating Trajectory')
    p0 = fa.get_pose()
    print(p0)
    p1 = p0.copy()
    trans_z = RigidTransform(
        translation=np.array([0, 0, 0.1]),
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
    
    trans_y = RigidTransform(
        translation=np.array([0, 0.1, 0]),
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
    
    trans_x = RigidTransform(
        translation=np.array([0.1, 0, 0]),
        rotation=RigidTransform.z_axis_rotation(np.deg2rad(0)), 
                            from_frame=p1.from_frame, to_frame=p1.from_frame)
    
    p1 = p1 * trans_z

    s = time.time()

    for i in range(3):           
        a = time.time()
        fa.goto_pose(p1, duration=1, use_impedance=True)
        b = time.time() - a
        print(b)

        if (i % 2 == 0):
            p1 = p1 * trans_y
        else:
            p1 = p1 * trans_z

    print(time.time() - s)
    rospy.loginfo('Done')
