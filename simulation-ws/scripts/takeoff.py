#!/usr/bin/env python3
import rospy
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardTakeoff:
    def __init__(self):
        self.ns = rospy.get_param("~ns", "/uav1")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.altitude_m = float(rospy.get_param("~altitude_m", 2.0))  # "para cima" em metros
        self.x = float(rospy.get_param("~x", 0.0))
        self.y = float(rospy.get_param("~y", 0.0))
        self.yaw = float(rospy.get_param("~yaw", 0.0))
        self.use_ned = bool(rospy.get_param("~use_ned", True))  # True => z negativo sobe

        self.state = State()
        rospy.Subscriber(f"{self.ns}/mavros/state", State, self.state_cb, queue_size=10)

        self.sp_pub = rospy.Publisher(
            f"{self.ns}/mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=10
        )

        rospy.wait_for_service(f"{self.ns}/mavros/cmd/arming")
        rospy.wait_for_service(f"{self.ns}/mavros/set_mode")
        self.arm_srv = rospy.ServiceProxy(f"{self.ns}/mavros/cmd/arming", CommandBool)
        self.mode_srv = rospy.ServiceProxy(f"{self.ns}/mavros/set_mode", SetMode)

        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.publish_setpoint)

    def state_cb(self, msg: State):
        self.state = msg

    def make_setpoint(self) -> PositionTarget:
        sp = PositionTarget()
        sp.header.stamp = rospy.Time.now()
        sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        # Usar somente posição + yaw
        sp.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )

        sp.position.x = self.x
        sp.position.y = self.y

        # NED: z positivo é "para baixo", então subir = z negativo
        sp.position.z = abs(self.altitude_m) if self.use_ned else abs(self.altitude_m)

        sp.yaw = self.yaw
        return sp

    def publish_setpoint(self, _evt):
        self.sp_pub.publish(self.make_setpoint())

    def run(self):
        rate = rospy.Rate(self.rate_hz)

        rospy.loginfo("Aguardando FCU conectar...")
        while not rospy.is_shutdown() and not self.state.connected:
            rate.sleep()

        rospy.loginfo("Publicando setpoints por 3s (pré-requisito OFFBOARD)...")
        t0 = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - t0).to_sec() < 3.0:
            rate.sleep()

        rospy.loginfo("Setando modo OFFBOARD...")
        resp = self.mode_srv(base_mode=0, custom_mode="OFFBOARD")
        rospy.loginfo(f"set_mode: mode_sent={resp.mode_sent}")

        rospy.sleep(0.5)

        rospy.loginfo("Armando...")
        resp = self.arm_srv(True)
        rospy.loginfo(f"arming: success={resp.success}")

        rospy.loginfo("Takeoff ativo. Mantendo setpoints (Ctrl+C para parar).")
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("uav1_offboard_takeoff")
    OffboardTakeoff().run()
