#!/usr/bin/env python3
import math, time, rclpy
from rclpy.node   import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32

MAX_LIN  = 3.0   # turtlesim limits
MAX_ANG  = 6.0

def clamp(val, low, high):
    return max(min(val, high), low)

class PController(Node):
    def __init__(self):
        super().__init__("p_controller")

        # ─ parameters ─
        self.declare_parameter("shape",  "triangle")
        self.declare_parameter("side",    2.0)
        self.declare_parameter("kp_ang",  3.0)
        self.declare_parameter("kp_lin",  1.2)

        # ─ internal state ─
        self.state       = 'ROTATE'   # ROTATE ▸ DRIVE
        self.waypoints   = None
        self.current_wp  = 0           # index of active vertex

        # pubs / subs
        self.pub = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.sub = self.create_subscription(
            Pose,  "/turtle1/pose", self.pose_cb, 10)

        # enable live tuning ------------------------------------------------
        self.add_on_set_parameters_callback(self._param_cb)

    # -------- pose callback ----------------------------------------------
    def pose_cb(self, pose: Pose):
        kp_ang = self.get_parameter("kp_ang").value
        kp_lin = self.get_parameter("kp_lin").value

        # On first call: initialize start and waypoints
        if self.waypoints is None:
            self.waypoints = self._make_vertices(pose)
            self.current_wp = 1        # index of the first move

        # Fetch current target
        tx, ty = self.waypoints[self.current_wp]
        dx, dy = tx - pose.x, ty - pose.y
        dist   = math.hypot(dx, dy)
        yaw_err = self._wrap(math.atan2(dy, dx) - pose.theta)

        # State machine
        twist = Twist()
        if self.state == 'ROTATE':
            twist.angular.z = clamp(kp_ang * yaw_err, -MAX_ANG, MAX_ANG)
            if abs(yaw_err) < 0.05:           # within 3°
                self.state = 'DRIVE'

        elif self.state == 'DRIVE':
            twist.linear.x = clamp(kp_lin * dist, 0.0, MAX_LIN)
            twist.angular.z = clamp(0.3 * kp_ang * yaw_err,
                                    -MAX_ANG, MAX_ANG)
            if dist < 0.05:                   # reached vertex
                self.current_wp = (self.current_wp + 1) % len(self.waypoints)
                self.state = 'ROTATE'
            elif abs(yaw_err) > 0.3:       # only re-pivot if > ~25°
                self.state = 'ROTATE'
                twist.linear.x = 0.0 # stop driving 

        self.pub.publish(twist)

    def _make_vertices(self, pose: Pose):
        """Equilateral triangle, side=s, CCW, vertex-0 at (x0,y0), vertex-1 due east."""
        s = self.get_parameter("side").value
        h = math.sqrt(3)/2 * s
        x0, y0 = pose.x, pose.y
        return [
            (x0,         y0),
            (x0 + s,     y0),
            (x0 + 0.5*s, y0 + h)
        ]

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
