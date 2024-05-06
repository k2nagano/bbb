from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Widget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.gridLayout = QGridLayout(self)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)

        self.label_ch1 = QLabel("Ch1", self)
        self.label_ch2 = QLabel("Ch2", self)
        self.label_ch3 = QLabel("Ch3", self)
        self.label_ch1.setAlignment(Qt.AlignCenter)
        self.label_ch2.setAlignment(Qt.AlignCenter)
        self.label_ch3.setAlignment(Qt.AlignCenter)

        self.horizontalSlider_ch1 = QSlider(self)
        self.horizontalSlider_ch1.setOrientation(Qt.Horizontal)
        self.horizontalSlider_ch1.setMaximum(3000)
        self.horizontalSlider_ch1.setValue(1500)
        self.horizontalSlider_ch1.setTickPosition(QSlider.TicksAbove)
        self.horizontalSlider_ch1.setTickInterval(100)

        self.horizontalSlider_ch2 = QSlider(self)
        self.horizontalSlider_ch2.setOrientation(Qt.Horizontal)
        self.horizontalSlider_ch2.setMaximum(3000)
        self.horizontalSlider_ch2.setValue(1500)
        self.horizontalSlider_ch2.setTickPosition(QSlider.TicksAbove)
        self.horizontalSlider_ch2.setTickInterval(100)

        self.horizontalSlider_ch3 = QSlider(self)
        self.horizontalSlider_ch3.setOrientation(Qt.Horizontal)
        self.horizontalSlider_ch3.setMaximum(3000)
        self.horizontalSlider_ch3.setValue(1500)
        self.horizontalSlider_ch3.setTickPosition(QSlider.TicksAbove)
        self.horizontalSlider_ch3.setTickInterval(100)

        self.label_ch1_value = QLabel("1500", self)
        self.label_ch1_value.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        self.label_ch2_value = QLabel("1500", self)
        self.label_ch2_value.setAlignment(Qt.AlignRight|Qt.AlignVCenter)
        self.label_ch3_value = QLabel("1500", self)
        self.label_ch3_value.setAlignment(Qt.AlignRight|Qt.AlignVCenter)

        self.pushButton_reset = QPushButton("Reset", self)

        self.gridLayout.addWidget(self.label_ch1, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.label_ch2, 1, 0, 1, 1)
        self.gridLayout.addWidget(self.label_ch3, 2, 0, 1, 1)
        self.gridLayout.addWidget(self.horizontalSlider_ch1, 0, 1, 1, 1)
        self.gridLayout.addWidget(self.horizontalSlider_ch2, 1, 1, 1, 1)
        self.gridLayout.addWidget(self.horizontalSlider_ch3, 2, 1, 1, 1)
        self.gridLayout.addWidget(self.label_ch1_value, 0, 2, 1, 1)
        self.gridLayout.addWidget(self.label_ch2_value, 1, 2, 1, 1)
        self.gridLayout.addWidget(self.label_ch3_value, 2, 2, 1, 1)
        self.gridLayout.addWidget(self.pushButton_reset, 3, 0, 1, 3)



def main():
    a = QApplication()
    w = Widget()
    w.show()
    a.exec_()


if __name__ == '__main__':
    main()
