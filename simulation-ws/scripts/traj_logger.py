
# traj_logger.py
import os
import json
import csv
from datetime import datetime

class TrajLogger:
    """
    Responsável por persistir trajetórias amostradas e metadados em CSV/JSON.
    Não depende de rospy (para ficar reutilizável e fácil de testar).
    """

    def __init__(self, log_dir, dt, rate_hz, in_topic, out_topic, coordinate_frame, type_mask):
        self.log_dir = os.path.expanduser(log_dir)
        os.makedirs(self.log_dir, exist_ok=True)

        self.dt = float(dt)
        self.rate_hz = float(rate_hz)
        self.in_topic = str(in_topic)
        self.out_topic = str(out_topic)
        self.coordinate_frame = int(coordinate_frame)
        self.type_mask = int(type_mask)

        self.traj_count = 0

    def _stamp_to_float(self, stamp):
        # stamp: rospy.Time-like (secs, nsecs)
        return float(stamp.secs) + float(stamp.nsecs) * 1e-9

    def _now_tag(self):
        return datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    def _pose_to_dict(self, pose_msg):
        # pose_msg: geometry_msgs/PoseStamped ou None
        if pose_msg is None:
            return None

        return {
            "frame_id": pose_msg.header.frame_id,
            "stamp": self._stamp_to_float(pose_msg.header.stamp),
            "position": {
                "x": pose_msg.pose.position.x,
                "y": pose_msg.pose.position.y,
                "z": pose_msg.pose.position.z,
            },
            "orientation": {
                "x": pose_msg.pose.orientation.x,
                "y": pose_msg.pose.orientation.y,
                "z": pose_msg.pose.orientation.z,
                "w": pose_msg.pose.orientation.w,
            },
        }

    def save(self, traj_msg, samples, pose_at_receive=None):
        """
        traj_msg: kr_planning_msgs/Trajectory
        samples: list[(x,y,z,yaw)] já amostrada
        pose_at_receive: PoseStamped mais recente (opcional)
        Retorna: (csv_path, json_path)
        """
        self.traj_count += 1
        seq = int(traj_msg.header.seq)
        tag = self._now_tag()

        base = f"traj_seq{seq:06d}_n{self.traj_count:03d}_{tag}"
        csv_path = os.path.join(self.log_dir, base + ".csv")
        json_path = os.path.join(self.log_dir, base + ".json")

        # CSV: i, t, x, y, z, yaw
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["i", "t", "x", "y", "z", "yaw"])
            for i, (x, y, z, yaw) in enumerate(samples):
                w.writerow([i, i * self.dt, x, y, z, yaw])

        meta = {
            "seq": seq,
            "traj_stamp": self._stamp_to_float(traj_msg.header.stamp),
            "traj_frame_id": traj_msg.header.frame_id,
            "dt": self.dt,
            "rate_hz": self.rate_hz,
            "coordinate_frame": self.coordinate_frame,
            "type_mask": self.type_mask,
            "in_topic": self.in_topic,
            "out_topic": self.out_topic,
            "num_primitives": len(traj_msg.primitives),
            "num_samples": len(samples),
            "pose_at_receive": self._pose_to_dict(pose_at_receive),
        }

        with open(json_path, "w") as f:
            json.dump(meta, f, indent=2)

        return csv_path, json_path
