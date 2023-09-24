import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class Test(Node):
    def __init__(self):
        super().__init__("a")
        self.pub = self.create_publisher(Float64, "/aaa_state", 10)
        self.sub = self.create_subscription(Float64, "/aaa_command", self.sub_callback, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64()
        msg.data = self.i * 0.01
        self.pub.publish(msg)
        self.get_logger().info(f"[PUB]{msg.data}")
        self.i += 1

    def sub_callback(self, msg):
        self.get_logger().info(f"[SUB]{msg.data}")


def main(args=None):
    rclpy.init(args=args)
    test = Test()
    rclpy.spin(test)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
