#!/usr/bin/env python3
import rospy
import math
from threading import Lock

from aeplanner.msg import aeplannerActionResult
from mrs_msgs.msg import ReferenceStamped

def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class AEPMRSSupervisor:
    def __init__(self):
        self.frame_id   = rospy.get_param("~frame_id", "uav1/gps_baro_origin")
        self.pub_rate   = rospy.get_param("~pub_rate", 10.0)
        self.hold_time  = rospy.get_param("~hold_time", 1.0)  # segundos segurando o setpoint
        self.min_z      = rospy.get_param("~min_z", 0.3)

        self.out_topic  = rospy.get_param("~out", "/uav1/control_manager/reference")
        self.in_topic   = rospy.get_param("~in", "/aeplanner/make_plan/result")

        self.pub = rospy.Publisher(self.out_topic, ReferenceStamped, queue_size=10)
        self.sub = rospy.Subscriber(self.in_topic, aeplannerActionResult, self.cb_result, queue_size=10)

        self.lock = Lock()
        self.latest_ref = None
        self.latest_ref_until = rospy.Time(0)

        rospy.loginfo("[aep_mrs_supervisor] listening %s, publishing %s frame=%s rate=%.1fHz hold=%.2fs",
                      self.in_topic, self.out_topic, self.frame_id, self.pub_rate, self.hold_time)

        rospy.Timer(rospy.Duration(1.0/self.pub_rate), self.timer_pub)

    def cb_result(self, msg: aeplannerActionResult):
        p = msg.result.pose.pose.position
        q = msg.result.pose.pose.orientation
        yaw = yaw_from_quat(q)

        ref = ReferenceStamped()
        ref.header.frame_id = self.frame_id
        ref.reference.position.x = p.x
        ref.reference.position.y = p.y
        ref.reference.position.z = max(p.z, self.min_z)
        ref.reference.heading = yaw

        with self.lock:
            self.latest_ref = ref
            self.latest_ref_until = rospy.Time.now() + rospy.Duration.from_sec(self.hold_time)

        rospy.loginfo("[aep_mrs_supervisor] new target: (%.2f %.2f %.2f) yaw=%.1fdeg is_clear=%s",
                      ref.reference.position.x, ref.reference.position.y, ref.reference.position.z,
                      yaw*57.2958, msg.result.is_clear)

    def timer_pub(self, _evt):
        with self.lock:
            if self.latest_ref is None:
                return
            if rospy.Time.now() > self.latest_ref_until:
                return
            msg = self.latest_ref

        msg.header.stamp = rospy.Time.now()
        self.pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("aep_mrs_supervisor", anonymous=False)
    AEPMRSSupervisor()
    rospy.spin()
