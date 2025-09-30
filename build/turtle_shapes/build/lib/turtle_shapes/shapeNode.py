import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ShapeNode(Node):
    def __init__(self):
        super().__init__('shape_node')
        self.publisher_ = self.create_publisher(String, 'selected_shape', 10)

        self.get_logger().info("ShapeNode started. Type a shape name or 'stop'...")

        # Timer to repeatedly ask for user input
        self.create_timer(0.5, self.ask_user)

        self._last_input = None

    def ask_user(self):
        if self._last_input is None:
            try:
                user_input = input("Enter shape\n1-infinity\n2-heart\n3-star\n4-stop\n")
                self._last_input = user_input.strip()
                msg = String()
                msg.data = self._last_input
                self.publisher_.publish(msg)
                self.get_logger().info(f"Published: {msg.data}")
                self._last_input = None
            except EOFError:
                pass  # in case of Ctrl+D

def main(args=None):
    rclpy.init(args=args)
    node = ShapeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
