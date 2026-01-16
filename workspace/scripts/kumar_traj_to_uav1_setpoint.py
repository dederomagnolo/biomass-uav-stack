#!/usr/bin/env python3
import math
import rospy
import os

from kr_planning_msgs.msg import Trajectory as KrTrajectory
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped

from traj_logger import TrajLogger

def eval_poly(c, t):
    acc = 0.0
    p = 1.0
    for ci in c:
        acc += ci * p
        p *= t
    return acc

class KumarTrajToUav1:
    def __init__(self):
        self.dt = float(rospy.get_param("~dt", 0.05))
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self.out_topic = rospy.get_param("~out", "/uav1/mavros/setpoint_raw/local")
        self.in_topic = rospy.get_param("~in", "/quadrotor/local_plan_server/traj")
        self.pose_topic = rospy.get_param("~pose", "/uav1/mavros/local_position/pose")

        self.coordinate_frame = int(rospy.get_param("~coordinate_frame", 1))  # LOCAL_NED
        self.type_mask = int(rospy.get_param("~type_mask", 3064))             # pos+yaw only

        # LOG: apenas setup
        log_dir = rospy.get_param("~log_dir", os.path.expanduser("~/traj_logs"))
        self.logger = TrajLogger(
            log_dir=log_dir,
            dt=self.dt,
            rate_hz=self.rate_hz,
            in_topic=self.in_topic,
            out_topic=self.out_topic,
            coordinate_frame=self.coordinate_frame,
            type_mask=self.type_mask,
        )
        self.last_pose_msg = None

        self.queue = []    # list of (x,y,z,yaw)
        self.idx = 0
        self.last_seq = None

        self.have_pose = False
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 0.0
        self.hold_yaw = 0.0

        self.pub = rospy.Publisher(self.out_topic, PositionTarget, queue_size=10)
        rospy.Subscriber(self.pose_topic, PoseStamped, self.pose_cb, queue_size=10)
        rospy.Subscriber(self.in_topic, KrTrajectory, self.traj_cb, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(1.0/self.rate_hz), self.tick)

        rospy.loginfo("Bridge+HOLD ON: %s -> %s | dt=%.3f rate=%.1f",
                      self.in_topic, self.out_topic, self.dt, self.rate_hz)

    def pose_cb(self, msg: PoseStamped):
        self.have_pose = True
        self.hold_x = msg.pose.position.x
        self.hold_y = msg.pose.position.y
        self.hold_z = msg.pose.position.z
        # yaw: mantém 0 por enquanto (ou você pode estimar do quaternion depois)
        # manter yaw fixo é suficiente para hover inicial
        self.hold_yaw = self.hold_yaw

    def traj_cb(self, msg: KrTrajectory):
        if self.last_seq == msg.header.seq:
            return
        self.last_seq = msg.header.seq

        samples = []
        for prim in msg.primitives:
            T = prim.t
            if T <= 0:
                continue
            t = 0.0
            while t < T:
                x = eval_poly(prim.cx, t)
                y = eval_poly(prim.cy, t)
                z = eval_poly(prim.cz, t)
                yaw = eval_poly(prim.cyaw, t)
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                samples.append((x, y, z, yaw))
                t += self.dt

        if not samples:
            rospy.logwarn("Trajectory received but produced 0 samples; ignoring.")
            return
        
        csv_path, json_path = self.logger.save(msg, samples, pose_at_receive=self.last_pose_msg)
        rospy.loginfo("Saved trajectory: %s | %s", csv_path, json_path)

        self.queue = samples
        self.idx = 0
        rospy.loginfo("Loaded new Kumar trajectory: %d samples (seq=%d)", len(samples), msg.header.seq)

    def publish_sp(self, x, y, z, yaw):
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = self.coordinate_frame
        sp.type_mask = self.type_mask
        sp.position.x = x
        sp.position.y = y
        sp.position.z = z
        sp.yaw = yaw
        self.pub.publish(sp)

    def tick(self, _evt):
        # Se tiver trajetória carregada, executa
        if self.queue:
            x, y, z, yaw = self.queue[self.idx]
            self.publish_sp(x, y, z, yaw)

            self.idx += 1
            if self.idx >= len(self.queue):
                # segura no último ponto
                self.idx = len(self.queue) - 1
            return

        # Sem trajetória: HOLD
        if self.have_pose:
            self.publish_sp(self.hold_x, self.hold_y, self.hold_z, self.hold_yaw)
        else:
            # Sem pose ainda, segura em 0,0,2 (ajuste se quiser)
            self.publish_sp(0.0, 0.0, 2.0, 0.0)

if __name__ == "__main__":
    rospy.init_node("kumar_traj_to_uav1_setpoint_hold")
    KumarTrajToUav1()
    rospy.spin()
