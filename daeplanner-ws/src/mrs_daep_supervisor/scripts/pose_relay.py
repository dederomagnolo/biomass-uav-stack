#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PoseRelay:
    def __init__(self):
        self.in_topic  = rospy.get_param("~in",  "/uav1/odometry/odom_main")
        self.out_topic = rospy.get_param("~out", "/pose")

        self.pub = rospy.Publisher(self.out_topic, PoseStamped, queue_size=10)

        self.sub = rospy.Subscriber(
            self.in_topic,
            Odometry,
            self.cb,
            queue_size=10,
            tcp_nodelay=True
        )

        rospy.loginfo("[pose_relay] %s -> %s", self.in_topic, self.out_topic)

    def cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header          # mantém frame_id e stamp
        ps.pose   = msg.pose.pose
        self.pub.publish(ps)

if __name__ == "__main__":
    rospy.init_node("pose_relay", anonymous=False)
    PoseRelay()
    rospy.spin()
