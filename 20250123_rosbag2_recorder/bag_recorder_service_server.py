import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.parameter import Parameter
from example_interfaces.srv import Trigger
import yaml
import os
import shutil
from datetime import datetime
from rosidl_runtime_py.utilities import get_message

class BagRecorderService(Node):
    def __init__(self, base_directory, config_path):
        super().__init__('bag_recorder_service')
        self.base_directory = base_directory
        self.config_path = config_path
        self.bag_directory = None
        self.writer = None
        self.subscribers = []

        # トピック設定を読み取る
        self.topics = self.load_topics_from_config()

        # サービスサーバーの作成
        self.start_service = self.create_service(Trigger, 'start_recording', self.start_recording_callback)
        self.stop_service = self.create_service(Trigger, 'stop_recording', self.stop_recording_callback)

        self.get_logger().info("BagRecorderService is ready.")

    def load_topics_from_config(self):
        # YAMLファイルからトピック情報を読み取る
        with open(self.config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config['topics']

    def create_bag_directory(self):
        # タイムスタンプを使って新しいディレクトリ名を生成
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_directory = os.path.join(self.base_directory, f"rosbag_{timestamp}")
        if os.path.exists(bag_directory):
            shutil.rmtree(bag_directory)
        return bag_directory

    def start_recording_callback(self, request, response):
        if self.writer is not None:
            response.success = False
            response.message = "Recording is already in progress."
            return response

        # 新しいbagディレクトリを作成
        self.bag_directory = self.create_bag_directory()

        try:
            # Writerを初期化
            self.writer = Writer(self.bag_directory)
            self.writer.open()

            # 各トピックにサブスクライブしてメッセージを記録
            for topic_name, msg_type_str in self.topics.items():
                # メッセージ型を取得
                try:
                    msg_type = get_message(msg_type_str)
                except (AttributeError, ModuleNotFoundError):
                    self.get_logger().warn(f"Could not find message type for {topic_name}")
                    continue

                # トピック接続を追加
                conn = self.writer.add_connection(topic_name, msg_type_str)
                self.subscribers.append(self.create_subscription(
                    msg_type,
                    topic_name,
                    lambda msg, t=topic_name, c=conn: self.callback(msg, t, c),
                    QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE)
                ))

            response.success = True
            response.message = f"Recording started. Bag directory: {self.bag_directory}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to start recording: {e}"
            self.get_logger().error(response.message)

        return response

    def stop_recording_callback(self, request, response):
        if self.writer is None:
            response.success = False
            response.message = "No recording in progress."
            return response

        # サブスクライバを削除
        for sub in self.subscribers:
            self.destroy_subscription(sub)
        self.subscribers = []

        # Writerを閉じる
        try:
            self.writer.close()
            self.writer = None
            response.success = True
            response.message = f"Recording stopped. Bag directory: {self.bag_directory}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop recording: {e}"
            self.get_logger().error(response.message)

        return response

    def callback(self, msg, topic_name, connection):
        # メッセージをCDR形式でシリアライズして記録
        timestamp = self.get_clock().now().nanoseconds
        serialized_data = serialize_cdr(msg, connection.msgtype)
        self.writer.write(connection, timestamp, serialized_data)
        self.get_logger().info(f"Recorded message on {topic_name} at {timestamp}")

def main(args=None):
    rclpy.init(args=args)

    base_directory = 'rosbag_mcap_output'
    config_path = 'config.yaml'  # トピック情報を記載したYAMLファイル

    recorder = BagRecorderService(base_directory, config_path)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

