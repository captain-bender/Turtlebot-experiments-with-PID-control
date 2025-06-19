
# analyze_odom.py
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomAnalyzer(Node):
    def __init__(self):
        super().__init__('odom_analyzer')

        # Will be initialised on first message
        self.targets   = []
        self.curr_edge = 0
        self.errors    = []
        self.side      = 2.0

        self.sub = self.create_subscription(
            Odometry, '/turtle1/odom', self.cb, 10)

    def _edge_error(self, x, y):
        """‖ point-to-segment distance for current edge ‖"""
        a = self.targets[self.curr_edge]
        b = self.targets[(self.curr_edge + 1) % len(self.targets)]
        ax, ay = a
        bx, by = b

        # projection of AP onto AB
        abx, aby = bx - ax, by - ay
        apx, apy = x  - ax, y  - ay
        ab_len2  = abx*abx + aby*aby
        t = max(0.0, min(1.0, (apx*abx + apy*aby) / ab_len2))
        px, py = ax + t*abx, ay + t*aby
        return math.hypot(x - px, y - py)

    # main callback
    def cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not self.targets:           # first msg → build vertices
            self.targets = self._make_triangle(x, y, self.side)

        err = self._edge_error(x, y)
        self.errors.append(err)

        # if we got within 5 cm of the next vertex ⇒ advance edge
        nx, ny = self.targets[(self.curr_edge + 1) % len(self.targets)]
        if math.hypot(nx - x, ny - y) < 0.05:
            self.curr_edge = (self.curr_edge + 1) % len(self.targets)

    def stop(self):
        if not self.errors:
            print("No data.")
            return
        mean = sum(self.errors)/len(self.errors)
        rms  = math.sqrt(sum(e*e for e in self.errors)/len(self.errors))
        print(f"\n--- Analysis Result ---")
        print(f"Samples:    {len(self.errors)}")
        print(f"Mean error: {mean:.3f} m")
        print(f"RMS error:  {rms:.3f} m")

    def _make_triangle(self, x0, y0, side):
        """Return 3 CCW vertices starting one side ahead of (x0,y0)."""
        h = math.sqrt(3)/2 * self.side
        return [
            (x0,           y0),
            (x0 + self.side, y0),
            (x0 + 0.5*self.side, y0 + h)
        ]

def main():
    rclpy.init()
    node = OdomAnalyzer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        # no rclpy.shutdown() here

if __name__=='__main__':
    main()
