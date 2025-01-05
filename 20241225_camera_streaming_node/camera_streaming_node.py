import os
import time
import datetime
import rclpy
from rclpy.node import Node
import socket
import threading
from collections import deque
import cv2
import numpy as np
from std_srvs.srv import SetBool, Trigger


def get_output_filename(output_dir: str = None, sub_dir: str = None, prefix: str = None, suffix: str = None, extension: str = None) -> str:
    # 現在時刻を取得
    now = datetime.datetime.now()
    # ミリ秒を含む形式で時刻をフォーマット
    formatted_time = now.strftime("%Y%m%d-%H%M%S") + f".{now.microsecond // 1000:03d}"

    filepath = os.getcwd()
    if output_dir is not None and len(output_dir)>0:
        filepath = output_dir
    if sub_dir is not None and len(sub_dir)>0:
        filepath=os.path.join(filepath, sub_dir)
    os.makedirs(filepath, exist_ok=True)
    # ファイル名を生成
    filename = formatted_time
    if prefix is not None and len(prefix)>0:
        filename = prefix + '_' +filename
    if suffix  is not None and len(suffix)>0:
        filename = filename+'_'+suffix
    if extension  is not None and len(extension)>0:
        filename = filename+'.'+extension
    filename=os.path.join(filepath,filename)
    print(filename)
    return filename

# メインノードクラス


class CameraStreamingNode(Node):

    def __init__(self):
        super().__init__('camera_streaming_node')

        self.udp_port = 5600
        self.output_directory = '.'
        self.recording_format = 'mp4'
        self.pipeline = f"udpsrc port={self.udp_port} ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        self.video_capture = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        self.frame = None
        self.lock = threading.Lock()
        maxlen = 10
        self.queue = deque(maxlen=maxlen)
        self.queue2 = deque(maxlen=maxlen)
        # self.queue = deque()
        # self.queue2 = deque()
        self.max_packet_size = 65471  # 65535 - 32 - 32
        self.is_playing = True  # 再生フラグ
        self.is_recording = False  # 録画フラグ
        self.fps = float(self.video_capture.get(cv2.CAP_PROP_FPS))
        self.width = int(self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(self.fps, self.width, self.height)
        self.fourcc = None
        self.video_writer = None

        # スレッドを開始
        threading.Thread(target=self.receive_thread, daemon=True).start()
        threading.Thread(target=self.play_thread, daemon=True).start()
        threading.Thread(target=self.record_thread, daemon=True).start()

        # サービスの設定
        self.srv_play = self.create_service(SetBool, '/toggle_play',
                                            self.toggle_play_callback)
        self.srv_record = self.create_service(SetBool, '/toggle_record',
                                              self.toggle_record_callback)
        self.srv_capture = self.create_service(Trigger, '/capture_frame',
                                               self.capture_frame_callback)

    def __del__(self):
        if self.video_capture is not None:
            self.video_capture.release()
        if self.video_writer is not None:
            self.video_writer.release()

    # 受信スレッド：UDPでH.264データを受信し、キューに入れる
    def receive_thread(self):
        while True:
            with self.lock:
                if self.video_capture is not None:
                    ret, frame = self.video_capture.read()
                    if ret:
                        self.frame = frame  # フレームを保持
                        if self.is_playing:
                            self.queue.append(frame)
                        if self.is_recording:
                            self.queue2.append(frame)
                        print(f"{len(self.queue)} {len(self.queue2)}")
                    else:
                        print('failed to read frame!')
                else:
                    print('not streaming!')
            time.sleep(1e-6)

    # 再生スレッド：キューからフレームを取得し、処理と送信
    def play_thread(self):
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.connect(("127.0.0.1", 5601))

        while True:
            frame = None
            if len(self.queue) > 0:
                frame = self.queue.popleft()
            # print(frame)
            if frame is None:
                self.get_logger().error("Failed to get frame to play")
                continue

            # フレームをJPEGにエンコード
            ret, jpeg_data = cv2.imencode('.jpg', frame)
            if not ret:
                self.get_logger().error("Failed to encode frame to play")
                continue

            # パケットの分割と送信
            jpeg_bytes = jpeg_data.tobytes()
            total_size = len(jpeg_bytes)

            if total_size > self.max_packet_size:
                num_packets = (total_size + self.max_packet_size -
                               1) // self.max_packet_size
                for i in range(num_packets):
                    packet_data = jpeg_bytes[i * self.max_packet_size:(i + 1) *
                                             self.max_packet_size]
                    header = i.to_bytes(4, 'big') + num_packets.to_bytes(
                        4, 'big')
                    udp_socket.send(header + packet_data)
            else:
                header = (0).to_bytes(4, 'big') + (1).to_bytes(4, 'big')
                udp_socket.send(header + jpeg_bytes)

    def record_thread(self):
        while (True):
            with self.lock:
                if self.video_writer is not None:
                    frame = None
                    if len(self.queue2) > 0:
                        frame = self.queue2.popleft()
                    if frame is not None:
                        # MKVファイルにフレームを保存
                        self.video_writer.write(frame)
                    else:
                        self.get_logger().error(
                            "Failed to get frame to record")
            time.sleep(1e-6)

    # 再生の開始/停止を切り替えるサービスのコールバック
    def toggle_play_callback(self, request, response):
        is_playing = request.data
        if is_playing == self.is_playing:
            response.success = True
            response.message = "camera streaming currently " + "active" if self.is_playing else "inactive"
            return response

        with self.lock:
            if is_playing:
                self.pipeline = f"udpsrc port={self.udp_port} ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
                self.video_capture = cv2.VideoCapture(self.pipeline,
                                                      cv2.CAP_GSTREAMER)
            else:
                self.pipeline = None
                if self.video_capture is not None:
                    self.video_capture.release()
                self.video_capture = None
                self.queue.clear()
                self.queue2.clear()
                self.frame = None
        self.is_playing = is_playing
        response.success = True
        response.message = "camera streaming " + "activated" if self.is_playing else "inactivated"
        return response

    # 録画の開始/停止を切り替えるサービスのコールバック
    def toggle_record_callback(self, request, response):
        print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
        # 再生中でないならエラーにして何もしない
        if self.is_playing is False:
            response.success = False
            response.message = "camera streaming currently inactive"
            return response

        is_recording = request.data
        if is_recording == self.is_recording:
            response.success = True
            response.message = "camera recording currently " + "active" if self.is_recording else "inactive"
            return response

        with self.lock:
            if is_recording:
                self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.fps = float(self.video_capture.get(cv2.CAP_PROP_FPS))
                self.width = int(
                    self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                self.height = int(
                    self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                print(self.fps, self.width, self.height)
                output_filename = get_output_filename(output_dir=self.output_directory, sub_dir='Video', extension='mp4')
                self.video_writer = cv2.VideoWriter(output_filename,
                                                    self.fourcc, self.fps,
                                                    (self.width, self.height))
            else:
                self.fourcc = None
                self.fps = None
                self.width = None
                self.height = None
                if self.video_writer is not None:
                    self.video_writer.release()
                self.video_writer = None
                self.queue2.clear()

        self.is_recording = is_recording
        response.success = True
        response.message = "camera recording " + "activated" if self.is_recording else "inactivated"
        return response

    # 現在のフレームをJPEGとして保存するサービスのコールバック
    def capture_frame_callback(self, request, response):
        print('bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb')
        frame = None
        with self.lock:
            frame = self.frame
        if frame is None:
            print('frame is None')
            response.success = False
            response.message = "Could not get frame to capture!"
            return response

        # フレームをJPEGにエンコードして保存
        # ret, jpeg_data = cv2.imencode('.jpg', frame)
        # if not ret:
        #     print('Failed to encode frame')
        #     response.success = False
        #     response.message = "Failed to encode frame!"
        #     return response

        output_filename = get_output_filename(output_dir=self.output_directory, sub_dir='Photo',extension='jpg')
        cv2.imwrite(output_filename, frame)
        response.success = True
        response.message = f"Captured frame saved as {output_filename}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
