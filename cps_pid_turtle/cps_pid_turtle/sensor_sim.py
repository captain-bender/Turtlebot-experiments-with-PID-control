#!/usr/bin/env python3
import math, random, rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from nav_msgs.msg   import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math

def quat_from_yaw(yaw: float):
    """Return geometry_msgs/Quaternion for zero roll/pitch and given yaw."""
    half = yaw * 0.5
    return Quaternion(x=0.0,
                      y=0.0,
                      z=math.sin(half),
                      w=math.cos(half))
# ------------------------- constants -----------------------------
WORLD_FRAME  = "world"
CHILD_FRAME  = "turtle1"
# -----------------------------------------------------------------
class SensorSim(Node):
    def __init__(self):
        super().__init__("sensor_sim")
        self.declare_parameter("noise_std", 0.02)           # 2 cm / rad / m s⁻²
        self.noise = self.get_parameter("noise_std").value

        self.prev_lin_vel = None
        self.prev_time    = None

        self.odom_pub = self.create_publisher(Odometry, "/turtle1/odom", 10)
        self.imu_pub  = self.create_publisher(Imu,      "/turtle1/imu",  10)
        self.pose_sub = self.create_subscription(Pose,  "/turtle1/pose",
                                                 self.pose_cb, 10)

    # ------------------------- callback ---------------------------
    def pose_cb(self, pose: Pose):
        stamp = self.get_clock().now().to_msg()

        # ◼ Odometry ------------------------------------------------
        odom             = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = WORLD_FRAME
        odom.child_frame_id  = CHILD_FRAME
        odom.pose.pose.position.x = pose.x + random.gauss(0, self.noise)
        odom.pose.pose.position.y = pose.y + random.gauss(0, self.noise)

        q = quat_from_yaw(pose.theta)

        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x  = pose.linear_velocity
        odom.twist.twist.angular.z = pose.angular_velocity
        self.odom_pub.publish(odom)

        # ◼ IMU -----------------------------------------------------
        imu             = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = CHILD_FRAME
        imu.orientation   = odom.pose.pose.orientation
        imu.angular_velocity.z   = pose.angular_velocity + random.gauss(0, self.noise)

        # simple finite-difference linear acceleration
        now   = self.get_clock().now()
        if self.prev_time is not None:
            dt = (now - self.prev_time).nanoseconds / 1e9
            lin_acc = (pose.linear_velocity - self.prev_lin_vel) / dt if dt else 0.0
        else:
            dt, lin_acc = 0.0, 0.0
        self.prev_lin_vel = pose.linear_velocity
        self.prev_time    = now

        imu.linear_acceleration.x = lin_acc + random.gauss(0, self.noise)
        self.imu_pub.publish(imu)
# -----------------------------------------------------------------
def main():
    rclpy.init()
    rclpy.spin(SensorSim())
