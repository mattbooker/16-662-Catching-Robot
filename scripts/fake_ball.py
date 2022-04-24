import rospy
from geometry_msgs.msg import PointStamped, Point
import numpy as np

def fake_ball():
    pub = rospy.Publisher('ball_pose', PointStamped, queue_size=10)
    rospy.init_node('fake_ball')
    rate = rospy.Rate(10) # 10hz

    init_time = rospy.Time.now().to_time()

    x = np.random.randint(low=30, high=65, size=4) / 100
    y = np.random.randint(low=-30, high=15, size=4) / 100 

    map_of_points = [Point(x[i], y[i], 0.3) for i in range(4)]

    print(map_of_points)

    while pub.get_num_connections == 0:
        rospy.loginfo("Waiting for subs")
        rospy.sleep(1)

    while not rospy.is_shutdown():

        elapsed_time = rospy.Time.now().to_time() - init_time

        counter = elapsed_time // 3
        pos = int(counter % 4)
        ball = map_of_points[pos]
        rospy.loginfo(pos)

        msg = PointStamped()
        msg.point = ball
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"

        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        fake_ball()
    except rospy.ROSInterruptException:
        pass