import sys
from PySide2.QtWidgets import *
from PySide2.QtCore import *
from PySide2.QtGui import *


class Widget(QWidget):
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        layout = QGridLayout()
        self.setLayout(layout)
        self.button_node1_upper_limit = QPushButton("1+++", self)
        self.button_node1_upper_page = QPushButton("1++", self)
        self.button_node1_upper_step = QPushButton("1+", self)
        self.button_node1_lower_step = QPushButton("1-", self)
        self.button_node1_lower_page = QPushButton("1--", self)
        self.button_node1_lower_limit = QPushButton("1---", self)
        self.button_node2_upper_limit = QPushButton("2+++", self)
        self.button_node2_upper_page = QPushButton("2++", self)
        self.button_node2_upper_step = QPushButton("2+", self)
        self.button_node2_lower_step = QPushButton("2-", self)
        self.button_node2_lower_page = QPushButton("2--", self)
        self.button_node2_lower_limit = QPushButton("2---", self)
        layout.addWidget(self.button_node1_upper_limit, 1, 0)
        layout.addWidget(self.button_node1_upper_page, 2, 0)
        layout.addWidget(self.button_node1_upper_step, 3, 0)
        layout.addWidget(self.button_node1_lower_step, 4, 0)
        layout.addWidget(self.button_node1_lower_page, 5, 0)
        layout.addWidget(self.button_node1_lower_limit, 6, 0)
        layout.addWidget(self.button_node2_upper_limit, 0, 1)
        layout.addWidget(self.button_node2_upper_page, 0, 2)
        layout.addWidget(self.button_node2_upper_step, 0, 3)
        layout.addWidget(self.button_node2_lower_step, 0, 4)
        layout.addWidget(self.button_node2_lower_page, 0, 5)
        layout.addWidget(self.button_node2_lower_limit, 0, 6)
        self.button_node1_upper_limit.clicked.connect(self.clicked)
        self.button_node1_upper_page.clicked.connect(self.clicked)
        self.button_node1_upper_step.clicked.connect(self.clicked)
        self.button_node1_lower_step.clicked.connect(self.clicked)
        self.button_node1_lower_page.clicked.connect(self.clicked)
        self.button_node1_lower_limit.clicked.connect(self.clicked)
        self.button_node2_upper_limit.clicked.connect(self.clicked)
        self.button_node2_upper_page.clicked.connect(self.clicked)
        self.button_node2_upper_step.clicked.connect(self.clicked)
        self.button_node2_lower_step.clicked.connect(self.clicked)
        self.button_node2_lower_page.clicked.connect(self.clicked)
        self.button_node2_lower_limit.clicked.connect(self.clicked)

    def clicked(self, event):
        print("clicked")
        if self.sender() == self.button_node1_upper_limit:
            print("1+++")
        if self.sender() == self.button_node1_upper_page:
            print("1++")
        if self.sender() == self.button_node1_upper_step:
            print("1+")
        if self.sender() == self.button_node1_lower_step:
            print("1-")
        if self.sender() == self.button_node1_lower_page:
            print("1--")
        if self.sender() == self.button_node1_lower_limit:
            print("1---")
        if self.sender() == self.button_node2_upper_limit:
            print("2+++")
        if self.sender() == self.button_node2_upper_page:
            print("2++")
        if self.sender() == self.button_node2_upper_step:
            print("2+")
        if self.sender() == self.button_node2_lower_step:
            print("2-")
        if self.sender() == self.button_node2_lower_page:
            print("2--")
        if self.sender() == self.button_node2_lower_limit:
            print("2---")


def main():
    a = QApplication(sys.argv)
    w = Widget()
    w.show()
    sys.exit(a.exec_())


if __name__ == "__main__":
    main()
