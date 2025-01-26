import rclpy
from rclpy.node import Node
from rosbags.rosbag2 import Writer
from rosbags.serde import serialize_cdr
from rosbags.typesys import get_types_from_idl, register_types
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy, QoSReliabilityPolicy
import os
import shutil
from datetime import datetime
from rosidl_runtime_py.utilities import get_message

class BagRecorder(Node):
    def __init__(self, base_directory):
        super().__init__('bag_recorder')

        # 出力ディレクトリの設定
        self.base_directory = base_directory
        self.bag_directory = self.create_bag_directory()

        # Writerの初期化前に既存ディレクトリを削除
        if os.path.exists(self.bag_directory):
            shutil.rmtree(self.bag_directory)

        # bagファイルへの書き込みのためのWriter
        try:
            self.writer = Writer(self.bag_directory)
            self.writer.open()
        except Exception as e:
            self.get_logger().error(f"Failed to open Writer: {e}")
            raise

        # 記録したい特定のトピックとメッセージ型を定義
        self.topics = {
            '/topic_int': 'std_msgs/msg/Int32',
            '/topic_str': 'std_msgs/msg/String'
        }

        # 各トピックにサブスクライブしてメッセージを記録
        self.subscribers = []
        for topic_name, msg_type_str in self.topics.items():
            # メッセージ型の取得
            try:
                msg_type = get_message(msg_type_str)
            except (AttributeError, ModuleNotFoundError):
                self.get_logger().warn(f"Could not find message type for {topic_name}")
                continue

            # メッセージ型のフルパスを解決し、接続を追加
            conn = self.writer.add_connection(topic_name, msg_type_str)
            self.subscribers.append(self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=topic_name, c=conn: self.callback(msg, t, c),
                QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST, reliability=QoSReliabilityPolicy.RELIABLE)
            ))

    def create_bag_directory(self):
        # タイムスタンプを使って新しいディレクトリ名を生成
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return os.path.join(self.base_directory, f"rosbag_{timestamp}")

    def callback(self, msg, topic_name, connection):
        # メッセージをCDR形式でシリアライズして記録
        timestamp = self.get_clock().now().nanoseconds
        serialized_data = serialize_cdr(msg, connection.msgtype)
        self.writer.write(connection, timestamp, serialized_data)
        self.get_logger().info(f"Recorded message on {topic_name} at {timestamp}")

    def close_bag(self):
        # bagファイルを閉じる
        try:
            self.writer.close()
        except Exception as e:
            self.get_logger().error(f"Failed to close Writer: {e}")

def main(args=None):
    rclpy.init(args=args)

    base_directory = 'rosbag_mcap_output'
    recorder = BagRecorder(base_directory)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close_bag()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
