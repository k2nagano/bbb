import time
import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger

class BagRecorderClient(Node):
    def __init__(self):
        super().__init__('bag_recorder_client')

        # サービスクライアントの初期化
        self.start_client = self.create_client(Trigger, 'start_recording')
        self.stop_client = self.create_client(Trigger, 'stop_recording')

        while not self.start_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'start_recording' service...")
        while not self.stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for 'stop_recording' service...")

    def send_start_request(self):
        req = Trigger.Request()
        future = self.start_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_stop_request(self):
        req = Trigger.Request()
        future = self.stop_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)

    client = BagRecorderClient()

    # 記録開始
    start_response = client.send_start_request()
    client.get_logger().info(f"Start Response: {start_response.success}, {start_response.message}")

    # 数秒後に記録停止
    client.get_logger().info("Waiting for 5 seconds before stopping recording...")
    time.sleep(5)

    # 記録停止
    stop_response = client.send_stop_request()
    client.get_logger().info(f"Stop Response: {stop_response.success}, {stop_response.message}")

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

