#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap

import tf2_ros
import tf2_geometry_msgs


class MrsDaepBridgeMin:
    def __init__(self):
        # Params
        self.uav_ns = rospy.get_param("~uav_ns", "/uav1").rstrip("/")
        self.world_frame = rospy.get_param("~world_frame", f"{self.uav_ns.strip('/')}/world_origin")

        self.odom_in = rospy.get_param("~odom_in", f"{self.uav_ns}/estimation_manager/odom_main")
        self.pose_out = rospy.get_param("~pose_out", "/pose")

        self.octomap_in = rospy.get_param("~octomap_in", f"{self.uav_ns}/octomap_server/octomap_local_full")
        self.octomap_out = rospy.get_param("~octomap_out", "/aeplanner/octomap_full")

        self.tf_timeout = rospy.get_param("~tf_timeout", 0.2)  # seconds

        # TF2
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers
        self.pub_pose = rospy.Publisher(self.pose_out, PoseStamped, queue_size=10)
        self.pub_octomap = rospy.Publisher(self.octomap_out, Octomap, queue_size=2)

        # Subscribers
        self.sub_odom = rospy.Subscriber(self.odom_in, Odometry, self.cb_odom, queue_size=20)
        self.sub_octomap = rospy.Subscriber(self.octomap_in, Octomap, self.cb_octomap, queue_size=2)

        rospy.loginfo("[mrs_daep_bridge_min] uav_ns: %s", self.uav_ns)
        rospy.loginfo("[mrs_daep_bridge_min] world_frame: %s", self.world_frame)
        rospy.loginfo("[mrs_daep_bridge_min] odom_in: %s -> pose_out: %s", self.odom_in, self.pose_out)
        rospy.loginfo("[mrs_daep_bridge_min] octomap_in: %s -> octomap_out: %s", self.octomap_in, self.octomap_out)

    def cb_octomap(self, msg: Octomap):
        # Relay direto (mantém header/frame do MRS, normalmente world_origin)
        self.pub_octomap.publish(msg)

    def cb_odom(self, msg: Odometry):
        # Converte nav_msgs/Odometry -> geometry_msgs/PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Se já está no world_frame, publica direto
        if pose.header.frame_id == self.world_frame:
            self.pub_pose.publish(pose)
            return

        # Caso contrário, tenta transformar via TF
        try:
            # lookup_transform(target, source, time)
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                pose.header.frame_id,
                pose.header.stamp,
                rospy.Duration(self.tf_timeout),
            )
            pose_t = tf2_geometry_msgs.do_transform_pose(pose, tf)
            pose_t.header.stamp = pose.header.stamp  # mantém tempo do sensor
            pose_t.header.frame_id = self.world_frame
            self.pub_pose.publish(pose_t)

        except Exception as e:
            rospy.logwarn_throttle(
                1.0,
                "[mrs_daep_bridge_min] TF falhou (%s -> %s). Publicando SEM transformar (frame=%s). Erro: %s",
                pose.header.frame_id, self.world_frame, pose.header.frame_id, str(e)
            )
            # Fallback: publica sem transformar (não é o ideal, mas ajuda debug)
            self.pub_pose.publish(pose)


def main():
    rospy.init_node("mrs_daep_bridge_min", anonymous=False)
    MrsDaepBridgeMin()
