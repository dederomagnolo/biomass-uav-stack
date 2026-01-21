#!/usr/bin/env python3
import math
import rospy
import actionlib

import tf2_ros
import tf2_geometry_msgs  # noqa: F401 (needed for PoseStamped transform)

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from octomap_msgs.msg import Octomap

from tf.transformations import euler_from_quaternion

from aeplanner.msg import aeplannerAction, aeplannerGoal
from mrs_msgs.srv import Vec4, Vec4Request


def yaw_from_quat(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class MrsDaepSupervisor:
    """
    Responsibilities:
      1) Relay MRS OctoMap -> /aeplanner/octomap_full
      2) Publish /pose in AEPlanner world_frame (transforming from odom frame using TF)
      3) (Optional) Run exploration loop: call /aeplanner/make_plan -> call /uavX/octomap_planner/goto
    """

    def __init__(self):
        # -------------------------
        # Params
        # -------------------------
        self.uav_ns = rospy.get_param("~uav_ns", "/uav1")

        # Frames: we strongly recommend aligning to MRS octomap frame: uavX/world_origin
        self.world_frame = rospy.get_param("~world_frame", f"{self.uav_ns.strip('/')}/world_origin")
        self.robot_frame = rospy.get_param("~robot_frame", f"{self.uav_ns.strip('/')}/fcu")

        # Topics
        self.odom_in = rospy.get_param("~odom_in", f"{self.uav_ns}/estimation_manager/odom_main")
        self.pose_out = rospy.get_param("~pose_out", "/pose")

        # Octomap relay
        self.octomap_in = rospy.get_param("~octomap_in", f"{self.uav_ns}/octomap_server/octomap_local_full")
        self.octomap_out = rospy.get_param("~octomap_out", "/aeplanner/octomap_full")

        # AEPlanner action
        self.aeplanner_action = rospy.get_param("~aeplanner_action", "/aeplanner/make_plan")

        # MRS goto service
        self.goto_srv = rospy.get_param("~goto_srv", f"{self.uav_ns}/octomap_planner/goto")

        # Behavior
        self.bridge_only = rospy.get_param("~bridge_only", True)  # start safe: only bridge/relay
        self.goal_replan_hz = float(rospy.get_param("~goal_replan_hz", 0.5))  # Hz (if not bridge_only)
        self.arrival_tolerance = float(rospy.get_param("~arrival_tolerance", 0.6))  # meters
        self.arrival_timeout = float(rospy.get_param("~arrival_timeout", 60.0))  # seconds

        # -------------------------
        # TF
        # -------------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # -------------------------
        # Publishers/Subscribers
        # -------------------------
        self.pose_pub = rospy.Publisher(self.pose_out, PoseStamped, queue_size=10)
        self.octomap_pub = rospy.Publisher(self.octomap_out, Octomap, queue_size=2)

        self.last_pose_world = None
        self.last_yaw_world = None

        rospy.Subscriber(self.odom_in, Odometry, self._cb_odom, queue_size=20)
        rospy.Subscriber(self.octomap_in, Octomap, self._cb_octomap, queue_size=2)

        # -------------------------
        # Clients
        # -------------------------
        self.ae_client = actionlib.SimpleActionClient(self.aeplanner_action, aeplannerAction)
        self.goto_client = None  # service proxy, initialized lazily

        self.actions_taken = 0

        rospy.loginfo("mrs_daep_supervisor starting with:")
        rospy.loginfo(f"  uav_ns        = {self.uav_ns}")
        rospy.loginfo(f"  world_frame   = {self.world_frame}")
        rospy.loginfo(f"  robot_frame   = {self.robot_frame}")
        rospy.loginfo(f"  odom_in       = {self.odom_in}")
        rospy.loginfo(f"  pose_out      = {self.pose_out}")
        rospy.loginfo(f"  octomap_in    = {self.octomap_in}")
        rospy.loginfo(f"  octomap_out   = {self.octomap_out}")
        rospy.loginfo(f"  ae_action     = {self.aeplanner_action}")
        rospy.loginfo(f"  goto_srv      = {self.goto_srv}")
        rospy.loginfo(f"  bridge_only   = {self.bridge_only}")

    # -------------------------
    # Callbacks
    # -------------------------
    def _cb_octomap(self, msg: Octomap):
        # Relay as-is. Frame must match AEPlanner world_frame for consistency.
        if msg.header.frame_id and msg.header.frame_id != self.world_frame:
            rospy.logwarn_throttle(
                2.0,
                f"[octomap relay] octomap frame_id='{msg.header.frame_id}' != world_frame='{self.world_frame}'. "
                "AEPlanner may behave incorrectly unless frames are consistent."
            )
        self.octomap_pub.publish(msg)

    def _cb_odom(self, msg: Odometry):
        # Build PoseStamped from odom and transform to world_frame
        pose_in = PoseStamped()
        pose_in.header = msg.header
        pose_in.pose = msg.pose.pose

        try:
            # Use the exact stamp when possible. If TF isn't available at that stamp, fall back to latest.
            pose_out = self.tf_buffer.transform(pose_in, self.world_frame, rospy.Duration(0.05))
        except Exception as e1:
            try:
                # fallback: transform using latest available TF
                pose_in.header.stamp = rospy.Time(0)
                pose_out = self.tf_buffer.transform(pose_in, self.world_frame, rospy.Duration(0.05))
            except Exception as e2:
                rospy.logwarn_throttle(
                    2.0,
                    f"[pose bridge] TF transform failed {pose_in.header.frame_id} -> {self.world_frame}: "
                    f"{str(e1)} | fallback failed: {str(e2)}"
                )
                return

        # Publish to /pose for AEPlanner
        self.pose_pub.publish(pose_out)

        self.last_pose_world = pose_out
        self.last_yaw_world = yaw_from_quat(pose_out.pose.orientation)

    # -------------------------
    # Helpers
    # -------------------------
    def _ensure_connections(self):
        # Ensure AEPlanner action server
        rospy.loginfo(f"Waiting for AEPlanner action server: {self.aeplanner_action}")
        if not self.ae_client.wait_for_server(timeout=rospy.Duration(10.0)):
            raise RuntimeError(f"AEPlanner action server not available: {self.aeplanner_action}")

        # Ensure goto service
        rospy.loginfo(f"Waiting for MRS goto service: {self.goto_srv}")
        rospy.wait_for_service(self.goto_srv, timeout=10.0)
        self.goto_client = rospy.ServiceProxy(self.goto_srv, Vec4)

        rospy.loginfo("All connections are up.")

    def _call_goto(self, x, y, z, yaw):
        req = Vec4Request()
        req.goal = [float(x), float(y), float(z), float(yaw)]
        resp = self.goto_client(req)
        if not resp.success:
            rospy.logwarn(f"[goto] rejected: {resp.message}")
        return resp.success, resp.message

    def _wait_until_arrived(self, x, y, z, timeout_s):
        if self.last_pose_world is None:
            rospy.logwarn("[arrive] no pose yet, cannot wait.")
            return False

        start = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > timeout_s:
                return False

            p = self.last_pose_world.pose.position
            dist = math.sqrt((p.x - x) ** 2 + (p.y - y) ** 2 + (p.z - z) ** 2)
            if dist <= self.arrival_tolerance:
                return True

            rate.sleep()

        return False

    # -------------------------
    # Main loop (optional exploration)
    # -------------------------
    def spin(self):
        self._ensure_connections()

        if self.bridge_only:
            rospy.loginfo("bridge_only=true -> relaying /octomap + publishing /pose only.")
            rospy.spin()
            return

        rospy.loginfo("bridge_only=false -> starting exploration loop (make_plan -> goto).")
        rate = rospy.Rate(max(self.goal_replan_hz, 0.1))

        while not rospy.is_shutdown():
            if self.last_pose_world is None:
                rospy.logwarn_throttle(2.0, "No pose in world_frame yet. Waiting...")
                rate.sleep()
                continue

            # Prepare goal for AEPlanner
            goal = aeplannerGoal()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = self.world_frame
            goal.actions_taken = int(self.actions_taken)

            self.ae_client.send_goal(goal)
            finished = self.ae_client.wait_for_result(timeout=rospy.Duration(10.0))
            if not finished:
                rospy.logwarn("[make_plan] timeout waiting for result")
                rate.sleep()
                continue

            res = self.ae_client.get_result()
            if res is None:
                rospy.logwarn("[make_plan] got empty result")
                rate.sleep()
                continue

            if not res.is_clear or res.tree_size <= 0:
                rospy.logwarn(f"[make_plan] no valid plan (is_clear={res.is_clear}, tree_size={res.tree_size})")
                rate.sleep()
                continue

            tx = res.pose.pose.position.x
            ty = res.pose.pose.position.y
            tz = res.pose.pose.position.z

            # Prefer yaw from result if valid; else keep current yaw
            yaw = None
            oq = res.pose.pose.orientation
            if abs(oq.w) > 1e-6 or abs(oq.x) > 1e-6 or abs(oq.y) > 1e-6 or abs(oq.z) > 1e-6:
                yaw = yaw_from_quat(oq)
            else:
                yaw = self.last_yaw_world if self.last_yaw_world is not None else 0.0

            ok, msg = self._call_goto(tx, ty, tz, yaw)
            if ok:
                arrived = self._wait_until_arrived(tx, ty, tz, self.arrival_timeout)
                if not arrived:
                    rospy.logwarn("[goto] commanded but not arrived within timeout")
                else:
                    self.actions_taken += 1
            else:
                rospy.logwarn(f"[goto] failed: {msg}")

            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mrs_daep_supervisor", anonymous=False)
    try:
        node = MrsDaepSupervisor()
        node.spin()
    except Exception as e:
        rospy.logfatal(f"mrs_daep_supervisor crashed: {str(e)}")
        raise
