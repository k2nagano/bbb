import os
import socket
import threading
import queue
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

# メインノードクラス


class StreamingNode(Node):
    def __init__(self):
        super().__init__('h264_streamer')
        self.queue = queue.Queue()
        self.max_packet_size = 65471  # 65535 - 32 - 32
        self.output_file = "output.mkv"
        self.udp_port = 5600
        self.pipeline = f"udpsrc port={self.udp_port} ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        self.capture = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        self.fps = float(self.capture.get(cv2.CAP_PROP_FPS))
        self.width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(self.fps, self.width, self.height)
        self.is_playing = True  # 再生フラグ
        self.is_recording = True  # 録画フラグ
        self.fourcc = cv2.VideoWriter_fourcc(*'X264')
        self.video_writer = cv2.VideoWriter(self.output_file, self.fourcc, self.fps, (self.width, self.height))

        # スレッドを開始
        threading.Thread(target=self.receive_thread, daemon=True).start()
        threading.Thread(target=self.play_thread, daemon=True).start()

        # サービスの設定
        self.srv_play = self.create_service(SetBool, 'toggle_play', self.toggle_play_callback)
        self.srv_record = self.create_service(SetBool, 'toggle_record', self.toggle_record_callback)
        self.srv_capture = self.create_service(Trigger, 'capture_frame', self.capture_frame_callback)

    # 受信スレッド：UDPでH.264データを受信し、キューに入れる

    def receive_thread(self):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(("", 5602))

        while True:
            data, _ = udp_socket.recvfrom(65536)  # 1フレーム分を受信
            self.queue.put(data)

    # 再生スレッド：キューからフレームを取得し、処理と送信
    def play_thread(self):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.connect(("127.0.0.1", 5601))

        while True:
            frame_data = self.queue.get()

            # フレームをJPEGにエンコード
            np_frame = np.frombuffer(frame_data, dtype=np.uint8)
            img = cv2.imdecode(np_frame, cv2.IMREAD_COLOR)
            ret, jpeg_data = cv2.imencode('.jpg', img)

            if not ret:
                self.get_logger().error("Failed to encode frame")
                continue

            # パケットの分割と送信
            jpeg_bytes = jpeg_data.tobytes()
            total_size = len(jpeg_bytes)

            if total_size > self.max_packet_size:
                num_packets = (total_size + self.max_packet_size - 1) // self.max_packet_size
                for i in range(num_packets):
                    packet_data = jpeg_bytes[i * self.max_packet_size:(i + 1) * self.max_packet_size]
                    header = i.to_bytes(4, 'big') + num_packets.to_bytes(4, 'big')
                    udp_socket.send(header + packet_data)
            else:
                header = (0).to_bytes(4, 'big') + (1).to_bytes(4, 'big')
                udp_socket.send(header + jpeg_bytes)

            # MKVファイルにフレームを保存
            self.video_writer.write(img)

    def __del__(self):
        self.video_writer.release()

    # 再生の開始/停止を切り替えるサービスのコールバック

    def toggle_play_callback(self, request, response):
        self.is_playing = request.data
        response.success = True
        response.message = "Playback started" if self.is_playing else "Playback stopped"
        return response

    # 録画の開始/停止を切り替えるサービスのコールバック
    def toggle_record_callback(self, request, response):
        self.is_recording = request.data
        response.success = True
        response.message = "Recording started" if self.is_recording else "Recording stopped"
        return response

    # 現在のフレームをJPEGとして保存するサービスのコールバック
    def capture_frame_callback(self, request, response):
        if not self.queue.empty():
            frame_data = self.queue.get()

            # フレームをJPEGにエンコードして保存
            np_frame = np.frombuffer(frame_data, dtype=np.uint8)
            img = cv2.imdecode(np_frame, cv2.IMREAD_COLOR)
            capture_filename = "capture_frame.jpg"
            cv2.imwrite(capture_filename, img)
            response.success = True
            response.message = f"Captured frame saved as {capture_filename}"
        else:
            response.success = False
            response.message = "No frame available to capture"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StreamingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
