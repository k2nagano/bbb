import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from PySide2.QtWidgets import *
from PySide2.QtCore import *


class Widget(QWidget):
    def __init__(self, parent=None):
        super(Widget, self).__init__(parent)
        rclpy.init(args=sys.argv)
        self.node = Node("test")
        self.pub = self.node.create_publisher(Float64, "/aaa_command", 10)
        self.sub = self.node.create_subscription(
            Float64, "/aaa_state", self.sub_callback, 10
        )
        # timer_period = 0.1  # seconds
        # self.timer = self.node.create_timer(timer_period, self.timer_callback)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_callback)
        self.timer.start(10)
        layout = QVBoxLayout()
        layout_node1 = QHBoxLayout()
        layout_node2 = QHBoxLayout()
        layout_slider = QHBoxLayout()
        layout.addLayout(layout_node1)
        layout.addLayout(layout_node2)
        layout.addLayout(layout_slider)
        self.setLayout(layout)
        label_node1 = QLabel("node1:", self)
        label_node2 = QLabel("node2:", self)
        self.label_node1_command = QLabel("0.00", self)
        self.label_node2_command = QLabel("0.00", self)
        self.label_node1_state = QLabel("0.00", self)
        self.label_node2_state = QLabel("0.00", self)
        self.label_node1_state.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.label_node2_state.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.label_node1_command.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        self.label_node2_command.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
        layout_node1.addWidget(label_node1)
        layout_node1.addWidget(self.label_node1_command)
        layout_node1.addWidget(self.label_node1_state)
        layout_node2.addWidget(label_node2)
        layout_node2.addWidget(self.label_node2_command)
        layout_node2.addWidget(self.label_node2_state)
        self.slider1 = QSlider(self)
        self.slider2 = QSlider(self)
        self.slider1.setOrientation(Qt.Vertical)
        self.slider2.setOrientation(Qt.Horizontal)
        # self.slider2.setInvertedAppearance(True)
        self.slider1.setMinimum(-100)
        self.slider1.setMaximum(100)
        self.slider2.setMinimum(-100)
        self.slider2.setMaximum(100)
        self.slider1.setSingleStep(1)
        self.slider1.setPageStep(10)
        self.slider2.setSingleStep(1)
        self.slider2.setPageStep(10)
        layout_slider.addWidget(self.slider1)
        layout_slider.addWidget(self.slider2, alignment=Qt.AlignTop)
        self.slider1.valueChanged[int].connect(self.slider1_valueChanged)
        self.slider2.valueChanged[int].connect(self.slider2_valueChanged)

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def sub_callback(self, msg):
        # print(f"{msg.data:.2f}")
        self.label_node1_state.setText(f"{msg.data:.2f}")
        # self.label_node1_state.update()
        # QApplication.processEvents()

    def timer_callback(self):
        # print("timer_callback")
        rclpy.spin_once(self.node)

    def slider1_valueChanged(self, val):
        print(val)
        msg = Float64()
        msg.data = val * 0.01
        self.pub.publish(msg)
        self.label_node1_command.setText(f"{msg.data:.2f}")

    def slider2_valueChanged(self, val):
        print(val)
        msg = Float64()
        msg.data = val * 0.01
        # self.pub.publish(msg)
        self.label_node2_command.setText(f"{msg.data:.2f}")


def main():
    a = QApplication(sys.argv)
    w = Widget()
    w.show()
    sys.exit(a.exec_())


if __name__ == "__main__":
    main()
