import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import gi
import cv2
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class CameraStreamingNode(Node):
    def __init__(self):
        super().__init__('camera_streaming_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # GStreamer初期化
        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            'udpsrc port=5000 ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw, format=I420 ! appsink name=sink'
        )
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.set_property('emit-signals', True)
        self.appsink.connect('new-sample', self.on_new_sample)
        
        # MP4ファイルに書き出すためのVideoWriterを初期化
        self.video_writer = cv2.VideoWriter('output.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30.0, (640, 480))

        # GStreamer パイプラインを再生状態に
        self.pipeline.set_state(Gst.State.PLAYING)
        
    def on_new_sample(self, sink):
        # 新しいフレームが到着するたびに呼ばれる
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        caps = sample.get_caps()
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            frame_data = np.frombuffer(map_info.data, np.uint8)
            height = caps.get_structure(0).get_value('height')
            width = caps.get_structure(0).get_value('width')

            # I420 を RGB に変換
            yuv_image = frame_data.reshape((height * 3 // 2, width))
            bgr_image = cv2.cvtColor(yuv_image, cv2.COLOR_YUV2BGR_I420)

            # ROS2 Image メッセージに変換してパブリッシュ
            ros_image_msg = self.bridge.cv2_to_imgmsg(bgr_image, encoding='bgr8')
            self.publisher_.publish(ros_image_msg)

            # フレームをMP4ファイルに保存
            self.video_writer.write(bgr_image)

            buf.unmap(map_info)
        
        return Gst.FlowReturn.OK

    def destroy_node(self):
        # ノード終了時にパイプラインを停止し、リソースを解放
        self.pipeline.set_state(Gst.State.NULL)
        self.video_writer.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

