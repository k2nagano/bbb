import rclpy
from rclpy.node import Node
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from std_msgs.msg import String  # 記録するメッセージタイプに応じて変更
from sensor_msgs.msg import Image  # 例としてImageメッセージも記録

class BagRecorder(Node):
    def __init__(self, bag_directory):
        super().__init__('bag_recorder')

        # rosbag2の設定
        self.writer = SequentialWriter()

        storage_options = StorageOptions(uri=bag_directory, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        self.writer.open(storage_options, converter_options)

        # 記録するトピックとメッセージタイプを指定
        self.topics = {
            '/chatter': String,
            '/camera/image': Image
        }

        # トピックをサブスクライブしてデータを記録
        for topic_name, msg_type in self.topics.items():
            self.writer.create_topic({
                'name': topic_name,
                'type': msg_type.__module__ + '/' + msg_type.__name__,
                'serialization_format': 'cdr'
            })
            self.create_subscription(
                msg_type, topic_name, lambda msg, t=topic_name: self.callback(msg, t), 10
            )

    def callback(self, msg, topic_name):
        # メッセージを受信して記録
        timestamp = self.get_clock().now().nanoseconds
        self.writer.write(topic_name, msg, timestamp)
        self.get_logger().info(f'Recorded message on {topic_name} at {timestamp}')

    def close(self):
        # 書き込み終了時に閉じる
        self.writer.close()

def main(args=None):
    rclpy.init(args=args)
    
    # 記録するbagファイルの保存ディレクトリを指定
    bag_directory = 'rosbag_data'
    recorder = BagRecorder(bag_directory)

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

