import sys
from PySide2.QtWidgets import *
from PySide2.QtCore import *
from PySide2.QtGui import *


class Widget(QWidget):
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        self.resize(100, 100)

    def mousePressEvent(self, event):
        self.is_active = True

    def mouseReleaseEvent(self, event):
        self.is_active = False

    def mouseMoveEvent(self, event):
        if not self.is_active:
            return
        x = event.pos().x()
        y = event.pos().y()
        if not (0 < x < 100 and 0 < y < 100):
            self.is_active = False
            return
        print(event.pos())


def main():
    a = QApplication(sys.argv)
    w = Widget()
    w.show()
    sys.exit(a.exec_())


if __name__ == "__main__":
    main()
