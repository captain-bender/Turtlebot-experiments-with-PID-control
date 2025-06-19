#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node   import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32

MAX_LIN  = 3.0   # turtlesim limits
MAX_ANG  = 6.0

class PController(Node):
    def __init__(self):
        super().__init__("p_controller")

        # ---------------- user‐tunable parameters ----------------
        self.declare_parameter("shape",  "square")   # square | triangle
        self.declare_parameter("side",    2.0)       # metres
        self.declare_parameter("kp_ang",  3.0)       # rad⁻¹  (slider)
        self.declare_parameter("kp_lin",  1.2)       # m⁻¹    (slider)
        # ---------------------------------------------------------

        self.waypoints   = None          # list[(x,y)]
        self.wp_idx      = 0
        self.prev_time   = time.time()

        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_cb, 10)
        self.pub = self.create_publisher  (Twist, "/turtle1/cmd_vel", 10)

        # enable live tuning ------------------------------------------------
        self.add_on_set_parameters_callback(self._param_cb)

        self.error_pub = self.create_publisher(Float32, "~/error/dist", 10)
        self.yaw_pub   = self.create_publisher(Float32, "~/error/yaw",  10)

    # -------- pose callback ----------------------------------------------
    def pose_cb(self, pose: Pose):
        if self.waypoints is None:                    # first pose → build vertices
            self.waypoints = self._make_vertices(pose)

        wp = self.waypoints[self.wp_idx]
        dx, dy   = wp[0] - pose.x, wp[1] - pose.y
        dist     = math.hypot(dx, dy)
        target_y = math.atan2(dy, dx)
        yaw_err  = self._wrap(target_y - pose.theta)

        dt = max(time.time() - self.prev_time, 1e-3)
        self.prev_time = time.time()

        kp_ang = self.get_parameter("kp_ang").value
        kp_lin = self.get_parameter("kp_lin").value

        twist = Twist()
        twist.angular.z = max(min(kp_ang * yaw_err,  MAX_ANG), -MAX_ANG)
        if abs(yaw_err) < 0.25:                       # drive only when aligned
            twist.linear.x  = max(min(kp_lin * dist, MAX_LIN),  0.0)
        self.pub.publish(twist)

        if dist < 0.05:                               # waypoint reached
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        
        self.error_pub.publish(Float32(data=dist))
        self.yaw_pub.publish(Float32(data=yaw_err))

    # -------- helpers -----------------------------------------------------
    def _make_vertices(self, pose: Pose):
        """Regular N-gon where _vertex 0_ is one side-length in front of start pose."""
        n   = 4 if self.get_parameter("shape").value == "square" else 3
        s   = self.get_parameter("side").value
        x, y, th = pose.x, pose.y, pose.theta
        verts = []
        for i in range(n):
            ang = th + i * 2*math.pi / n   # CCW
            verts.append((x + s*math.cos(ang),
                          y + s*math.sin(ang)))
        return verts

    @staticmethod
    def _wrap(a):                         # normalise (-π, π]
        return math.atan2(math.sin(a), math.cos(a))

    # live-parameter callback
    def _param_cb(self, params):
        # Accept all changes (no validation needed here)
        return SetParametersResult(successful=True)

# entry point
def main():
    rclpy.init()
    node = PController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
