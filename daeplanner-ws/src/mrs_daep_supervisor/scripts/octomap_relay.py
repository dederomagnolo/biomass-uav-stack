#!/usr/bin/env python3
import rospy
from octomap_msgs.msg import Octomap

class OctomapRelay:
    def __init__(self):
        self.in_topic  = rospy.get_param("~in",  "/uav1/octomap_server/octomap_local_full")
        self.out_topic = rospy.get_param("~out", "/aeplanner/octomap_full")

        self.pub = rospy.Publisher(self.out_topic, Octomap, queue_size=1)

        # IMPORTANTÍSSIMO: buff_size grande para mensagens grandes (Octomap costuma ser grande)
        self.sub = rospy.Subscriber(
            self.in_topic,
            Octomap,
            self.cb,
            queue_size=1,
            buff_size=2**24,   # 16MB
            tcp_nodelay=True
        )

        self.count = 0
        rospy.loginfo("[octomap_relay] %s -> %s", self.in_topic, self.out_topic)

    def cb(self, msg: Octomap):
        self.pub.publish(msg)
        self.count += 1
        if self.count % 10 == 0:
            rospy.loginfo("[octomap_relay] relayed %d msgs (frame=%s)", self.count, msg.header.frame_id)

if __name__ == "__main__":
    rospy.init_node("octomap_relay", anonymous=False)
    OctomapRelay()
    rospy.spin()
