from PySide2.QtWidgets import QWidget, QApplication
from PySide2.QtCore import QPropertyAnimation, Qt, Property
from PySide2.QtGui import QPainter, QColor, QFont
import sys

class SlideButton(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(100, 40)  # ウィジェットサイズを少し広く設定
        self._button_pos = 0  # ボタンの初期位置
        self.is_on = False  # ON/OFF状態
        self.animation = QPropertyAnimation(self, b"button_pos")  # アニメーション

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing |
                               QPainter.HighQualityAntialiasing)

        # 背景色 (ONの時は緑、OFFの時は赤)
        if self.is_on:
            painter.setBrush(QColor(0, 200, 0))  # 緑色 (ON)
        else:
            painter.setBrush(QColor(200, 0, 0))  # 赤色 (OFF)
        painter.setPen(Qt.NoPen)
        painter.drawRoundedRect(0, 0, self.width(), self.height(), 20, 20)

        # ボタンの描画
        painter.setBrush(Qt.white)
        painter.drawEllipse(self._button_pos, 5, 30, 30)  # ボタン部分

        # テキストの描画
        painter.setPen(Qt.white)
        painter.setFont(QFont("Arial", 12, QFont.Bold))

        if self.is_on:
            painter.drawText(self.rect(), Qt.AlignCenter, "ON")  # ONの場合、中央にONと表示
        else:
            painter.drawText(self.rect(), Qt.AlignCenter, "OFF")  # OFFの場合、中央にOFFと表示

    def mousePressEvent(self, event):
        self.toggle()  # クリック時にトグル状態を変更

    def toggle(self):
        self.is_on = not self.is_on

        # アニメーションの設定
        self.animation.setDuration(200)  # アニメーションの長さ
        self.animation.setStartValue(self._button_pos)
        self.animation.setEndValue(self.width() - 35 if self.is_on else 0)  # ONなら右、OFFなら左へ
        self.animation.start()  # アニメーション開始

    # プロパティとして button_pos を設定
    @Property(int)
    def button_pos(self):
        return self._button_pos

    @button_pos.setter
    def button_pos(self, pos):
        self._button_pos = pos
        self.update()  # 再描画

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Slide Button with ON/OFF Text")
        self.setFixedSize(200, 100)

        self.slide_button = SlideButton(self)
        self.slide_button.move(50, 30)  # 中央にボタンを配置

if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec_())

