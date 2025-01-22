import sys
import cv2
import numpy as np
from PySide2.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QSlider, QSpinBox, QLabel
)
from PySide2.QtCore import Qt, QThread, Signal, Slot, QTimer, QMutex, QMutexLocker, QPointF
from PySide2.QtGui import QPainter, QPen
from math import pi, cos, sin
import threading


# SonarThreadクラス：OpenCVを使用してMKVファイルを別スレッドで処理する
class SonarThread(QThread):
    video_changed = Signal(str)  # 再生中のビデオファイルが変更されたときのシグナル
    frame_count_changed = Signal(int)  # MKVの総フレーム数が変わったとき
    frame_ready = Signal(np.ndarray)  # 指定されたフレームが準備できたときのシグナル

    def __init__(self, video_path):
        super().__init__()
        self.video_path = video_path
        self.lock = threading.Lock()
        self.cap = cv2.VideoCapture(video_path)
        self.state = 'stopped'
        self.skip_frames = 1
        self.current_frame = 0
        self.latest_frame = None
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

    def run(self):
        while self.cap.isOpened():
            with self.lock:
                if self.state == 'playing':
                    ret, frame = self.cap.read()
                    if ret:
                        self.current_frame += self.skip_frames
                        self.latest_frame = frame  # 最新フレームを保持
                        self.frame_ready.emit(frame)  # フレームを準備完了として通知
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, self.current_frame)
                    else:
                        break
            self.msleep(50)  # 適切な間隔を設定

    def get_latest_frame(self):
        with self.lock:
            return self.latest_frame

    def play(self):
        self.state = 'playing'

    def pause(self):
        self.state = 'paused'

    def stop(self):
        self.state = 'stopped'

    def fast_forward(self):
        self.state = 'fast_forward'
        self.skip_frames = 30

    def rewind(self):
        self.state = 'rewind'
        self.skip_frames = -30

    def set_position(self, pos):
        # 指定された位置のフレームを取得し、frame_readyシグナルで通知する
        with self.lock:
            self.current_frame = pos
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, pos)
            ret, frame = self.cap.read()
            if ret:
                self.latest_frame = frame
                self.frame_ready.emit(frame)

    def set_video(self, video_path):
        with self.lock:
            self.video_path = video_path
            self.cap.release()
            self.cap = cv2.VideoCapture(video_path)
            self.current_frame = 0
            self.state = 'paused'
            self.latest_frame = None
            self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.video_changed.emit(video_path)
        self.frame_count_changed.emit(self.total_frames)

    def terminate(self):
        with self.lock:
            self.cap.release()
        super().terminate()


# SonarWidgetクラス：画面に扇形を描画する
class SonarWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.frame = None
        self.max_range = 1000  # デフォルトの最大距離

    def set_frame(self, frame):
        self.frame = frame
        self.update()

    def set_max_range(self, range_m):
        self.max_range = range_m
        self.update()

    def paintEvent(self, event):
        if self.frame is None:
            return

        # フレームをグレースケールに変換して描画
        gray_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        painter = QPainter(self)
        painter.setPen(QPen(Qt.black, 2))

        # 扇形を描画する
        center = self.rect().center()
        radius = min(self.rect().width(), self.rect().height()) / 2 * (self.max_range / 1000.0)
        num_lines = 20  # 扇形の分割数

        for i in range(num_lines):
            angle = (pi / num_lines) * i - (pi / 2)
            intensity = gray_frame[gray_frame.shape[0] // 2, i * gray_frame.shape[1] // num_lines]
            end_point = QPointF(
                center.x() + radius * cos(angle) * (intensity / 255.0),
                center.y() + radius * sin(angle) * (intensity / 255.0)
            )
            painter.drawLine(center, end_point)

        # 描画終了処理
        painter.end()


# SonarPlayerクラス：再生、停止、早送りなどの機能を持つ
class SonarPlayer(QWidget):
    def __init__(self, video_path, max_range):
        super().__init__()

        # GUIパーツの設定
        self.sonar_widget = SonarWidget()
        self.play_button = QPushButton("Play")
        self.pause_button = QPushButton("Pause")
        self.ff_button = QPushButton("Fast Forward")
        self.rewind_button = QPushButton("Rewind")
        self.stop_button = QPushButton("Stop")
        self.range_spinbox = QSpinBox()
        self.range_spinbox.setRange(100, 2000)
        self.range_spinbox.setValue(int(max_range * 1000))  # 初期値を引数の`max_range`に設定
        self.set_max_range_button = QPushButton("Set max range")
        self.slider = QSlider(Qt.Horizontal)
        self.status_label = QLabel(f"Now playing: {video_path}")

        # レイアウトの設定
        button_layout = QHBoxLayout()
        button_layout.addWidget(self.play_button)
        button_layout.addWidget(self.pause_button)
        button_layout.addWidget(self.ff_button)
        button_layout.addWidget(self.rewind_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.range_spinbox)
        button_layout.addWidget(self.set_max_range_button)  # [Set max range]ボタン追加

        main_layout = QVBoxLayout()
        main_layout.addWidget(self.sonar_widget)
        main_layout.addLayout(button_layout)
        main_layout.addWidget(self.slider)
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)

        # ドラッグ＆ドロップを有効にする
        self.setAcceptDrops(True)

        # SonarThreadの設定
        self.sonar_thread = SonarThread(video_path)
        self.sonar_thread.video_changed.connect(self.update_status)
        self.sonar_thread.frame_count_changed.connect(self.update_slider_range)
        self.sonar_thread.frame_ready.connect(self.sonar_widget.set_frame)  # フレームを表示
        self.sonar_thread.start()

        # ボタンのシグナル接続
        self.play_button.clicked.connect(self.sonar_thread.play)
        self.pause_button.clicked.connect(self.sonar_thread.pause)
        self.ff_button.clicked.connect(self.sonar_thread.fast_forward)
        self.rewind_button.clicked.connect(self.sonar_thread.rewind)
        self.stop_button.clicked.connect(self.sonar_thread.stop)
        self.set_max_range_button.clicked.connect(self.update_max_range)  # Set max rangeボタンの接続
        self.slider.sliderPressed.connect(self.pause_video)  # スライダー操作中は一時停止
        self.slider.sliderReleased.connect(self.resume_video)  # スライダー操作後に再生再開
        self.slider.valueChanged.connect(self.sonar_thread.set_position)

        # 定期的にスレッドからフレームを取得して表示するためのタイマー設定
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(100)  # 適切な周期で更新

    @Slot()
    def update_frame(self):
        frame = self.sonar_thread.get_latest_frame()
        if frame is not None:
            self.sonar_widget.set_frame(frame)

    @Slot(str)
    def update_status(self, video_path):
        self.status_label.setText(f"Now playing: {video_path}")

    @Slot(int)
    def update_slider_range(self, total_frames):
        self.slider.setRange(0, total_frames - 1)

    def pause_video(self):
        self.sonar_thread.pause()  # スライダーを操作する際は再生を一時停止

    def resume_video(self):
        self.sonar_thread.play()  # スライダーの操作が終わったら再生再開

    @Slot()
    def update_max_range(self):
        max_range = self.range_spinbox.value()
        self.sonar_widget.set_max_range(max_range)

    # ドラッグ＆ドロップで新しいビデオファイルを設定
    def dragEnterEvent(self, event):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()

    def dropEvent(self, event):
        urls = event.mimeData().urls()
        if urls:
            file_path = urls[0].toLocalFile()
            if file_path.endswith(".mkv"):  # MKVファイルのみを受け入れる
                self.sonar_thread.set_video(file_path)
                self.slider.setValue(0)

    def closeEvent(self, event):
        # アプリケーション終了時にスレッドを停止させる
        self.sonar_thread.terminate()
        self.sonar_thread.wait()  # スレッドが完全に終了するまで待機
        event.accept()


# メイン関数
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # コマンドライン引数からmax_rangeとmkvファイルを取得
    import argparse
    parser = argparse.ArgumentParser(description="Sonar Player with max range and MKV file input")
    parser.add_argument("mkv_file", help="Path to the MKV file")
    parser.add_argument("--max_range", type=float, default=3.0, help="Max range in meters (default: 3.0)")
    args = parser.parse_args()

    video_path = args.mkv_file
    max_range = args.max_range

    player = SonarPlayer(video_path, max_range)
    player.resize(800, 600)  # ウィンドウサイズを調整
    player.show()

    sys.exit(app.exec_())

