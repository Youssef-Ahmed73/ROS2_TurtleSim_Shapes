#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleCommander(Node):
    def __init__(self):
        super().__init__('turtle_commander')
        self.sub_shape = self.create_subscription(String, 'selected_shape', self.shape_cb, 10)
        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.pub_vel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info("TurtleCommander ready. Waiting for shape selections...")

        self.current_shape = None
        self.t = 0.0
        self.pose = None
        self.last_time = self.get_clock().now()

        # timer runs the controller
        self.create_timer(0.05, self.update)  # 20 Hz

        # controller gains / limits
        self.k_lin = 1.2
        self.k_ang = 6.0
        self.max_lin = 2.0
        self.max_ang = 6.0

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def shape_cb(self, msg: String):
        s = msg.data.strip().lower()
        if s in ("1", "infinity", "infty", "∞"):
            self.current_shape = "infinity"
        elif s in ("2", "heart"):
            self.current_shape = "heart"
        elif s in ("3", "star"):
            self.current_shape = "star"
        elif s in ("4", "stop", "0", "none"):
            self.current_shape = None
        else:
            self.get_logger().warn(f"Unknown selection: {msg.data}")
            return
        self.t = 0.0
        self.get_logger().info(f"Selected shape: {self.current_shape}")

    def trajectory_point(self, t: float, shape: str):
        """
        Return (x,y) target point in turtlesim coordinates (approx 0..11).
        Centered near (5.5,5.5).
        """
        cx, cy = 5.5, 5.5
        if shape == "infinity":
            # Lemniscate of Bernoulli (scaled)
            a = 3.0
            denom = 1.0 + math.sin(t)**2
            x = a * math.cos(t) / denom
            y = a * math.cos(t) * math.sin(t) / denom
            return cx + x, cy + y

        if shape == "heart":
            # Classic heart parametric (scaled down)
            x = 16 * math.sin(t)**3
            y = 13 * math.cos(t) - 5 * math.cos(2*t) - 2 * math.cos(3*t) - math.cos(4*t)
            scale = 0.10  # tune to fit screen
            return cx + scale * x, cy + scale * y

        if shape == "star":
            # 5-pointed star (x,y) vertices
            R = 3.0
            cx, cy = 5.5, 5.5
            star_points = []
            for i in range(5):
                ang = i * 4.0 * math.pi / 5.0  # 144° increments
                x = cx + R * math.cos(ang)
                y = cy + R * math.sin(ang)
                star_points.append((x, y))
            # move along segments
            loop = 5.0  # 5 segments
            u = (t % loop)
            i = int(u) % 5
            f = u - int(u)
            x1, y1 = star_points[i]
            x2, y2 = star_points[(i + 2) % 5]  # jump to every other point to make star
            return x1 + (x2 - x1) * f, y1 + (y2 - y1) * f

        # default center
        return cx, cy

    def update(self):
        # Stop if no shape selected or we don't have current pose
        if not self.current_shape or self.pose is None:
            # publish zero twist to stop turtle
            self.pub_vel.publish(Twist())
            self.last_time = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.05
        self.last_time = now
        self.t += dt * 0.8  # speed factor (tune if shape runs too fast/slow)

        tx, ty = self.trajectory_point(self.t, self.current_shape)

        # vector to target
        dx = tx - self.pose.x
        dy = ty - self.pose.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_goal - self.pose.theta)

        twist = Twist()
        # angular velocity proportional to angle_diff
        twist.angular.z = max(-self.max_ang, min(self.max_ang, self.k_ang * angle_diff))
        # linear velocity proportional to distance but zero if angle too large
        if abs(angle_diff) < 0.6:
            twist.linear.x = max(0.0, min(self.max_lin, self.k_lin * distance))
        else:
            twist.linear.x = 0.0

        self.pub_vel.publish(twist)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
