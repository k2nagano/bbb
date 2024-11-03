import numpy as np
from PySide2 import QtWidgets
from PySide2 import QtCore
from PySide2 import QtGui


class Widget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout()
        push_button = QtWidgets.QPushButton()
        self.line_edit = QtWidgets.QLineEdit()
        layout.addWidget(push_button)
        layout.addWidget(self.line_edit)
        self.setLayout(layout)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.callback)
        self.timer.setInterval(100)
        self.timer.start()
        print("000")
        self.value = 0.
        self.line_edit_palette = self.line_edit.palette()

    def callback(self):
        print("aaa")
        self.value += 0.01
        if self.value > 1. - 1e-6:
            self.timer.stop()
            self.line_edit.setText("Completed!")
            self.line_edit.setPalette(self.line_edit_palette)
            return
        # self.line_edit.setStyleSheet(
        #     "QLineEdit{background: qlineargradient(x1: 0, y1: 0, x2: 1, y2: 0,    stop: 0 #D3D3D3, stop: 0.4 #D8D8D8,                                stop: 0.5 #DDDDDD, stop: 1.0 #E1E1E1);}")
        self.line_edit.setText(f'progress {self.value:.2f}')
        # palette = self.lineedit.palette()
        palette = QtGui.QPalette()
        QRectF = QtCore.QRectF(self.line_edit.rect())
        gradient = QtGui.QLinearGradient(QRectF.topLeft(), QRectF.topRight())
        gradient.setColorAt(self.value-0.001, QtGui.QColor('#f99e41'))
        gradient.setColorAt(self.value, QtGui.QColor('#ffffff'))
        gradient.setColorAt(self.value+0.001, QtGui.QColor('#ffffff'))
        palette.setBrush(QtGui.QPalette.Base, QtGui.QBrush(gradient))
        self.line_edit.setPalette(palette)

def main():
    a = QtWidgets.QApplication([])
    w = Widget()
    w.show()
    a.exec_()


if __name__ == '__main__':
    main()
