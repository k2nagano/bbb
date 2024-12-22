import cv2
import socket
import struct
import threading

# スレッド1: UDPでH.264ストリーミングを受信
class FrameReceiver(threading.Thread):
    def __init__(self, udp_port):
        super().__init__()
        self.udp_port = udp_port
        self.pipeline = f"udpsrc port={self.udp_port} ! application/x-rtp, payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink"
        self.capture = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        self.running = True
        self.frame = None
        self.lock = threading.Lock()

    def run(self):
        while self.running:
            ret, frame = self.capture.read()
            if ret:
                with self.lock:
                    self.frame = frame  # フレームを保持

    def stop(self):
        self.running = False
        self.capture.release()

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None


# スレッド2: フレームをUDPで送信
class FrameSender(threading.Thread):
    def __init__(self, send_port, receiver_thread):
        super().__init__()
        self.send_port = send_port
        self.receiver_thread = receiver_thread
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        self.udp_target = ("127.0.0.1", self.send_port)  # 送信先のポート

    def run(self):
        while self.running:
            frame = self.receiver_thread.get_frame()
            if frame is not None:
                height, width, _ = frame.shape
                # フレームをRGB888に変換
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # フォーマットに従ってバイトデータを作成
                width_bytes = struct.pack('!H', width)  # uint16の幅
                height_bytes = struct.pack('!H', height)  # uint16の高さ
                rgb_data = rgb_frame.tobytes()  # 画像データをバイト列に変換

                # 幅、高さ、RGBデータを結合
                packet = width_bytes + height_bytes + rgb_data

                # UDPでデータ送信
                self.sock.sendto(packet, self.udp_target)

    def stop(self):
        self.running = False
        self.sock.close()


def main():
    # フレーム受信スレッド（ポート5600でH.264ストリームを受信）
    receiver_thread = FrameReceiver(udp_port=5600)
    receiver_thread.start()

    # フレーム送信スレッド（RGB888形式にしてポート5601に送信）
    sender_thread = FrameSender(send_port=5601, receiver_thread=receiver_thread)
    sender_thread.start()

    try:
        while True:
            pass  # メインスレッドはスレッド間の調整だけ行う
    except KeyboardInterrupt:
        print("Shutting down...")
    
    # 終了処理
    receiver_thread.stop()
    sender_thread.stop()
    receiver_thread.join()
    sender_thread.join()


if __name__ == "__main__":
    main()

