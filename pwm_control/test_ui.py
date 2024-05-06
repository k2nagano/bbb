# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'test.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(400, 300)
        self.gridLayoutWidget = QWidget(Form)
        self.gridLayoutWidget.setObjectName(u"gridLayoutWidget")
        self.gridLayoutWidget.setGeometry(QRect(90, 100, 160, 105))
        self.gridLayout = QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.label_ch3 = QLabel(self.gridLayoutWidget)
        self.label_ch3.setObjectName(u"label_ch3")
        self.label_ch3.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_ch3, 1, 0, 1, 1)

        self.label_ch2 = QLabel(self.gridLayoutWidget)
        self.label_ch2.setObjectName(u"label_ch2")
        self.label_ch2.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_ch2, 2, 0, 1, 1)

        self.horizontalSlider_ch2 = QSlider(self.gridLayoutWidget)
        self.horizontalSlider_ch2.setObjectName(u"horizontalSlider_ch2")
        self.horizontalSlider_ch2.setMaximum(3000)
        self.horizontalSlider_ch2.setValue(1500)
        self.horizontalSlider_ch2.setOrientation(Qt.Horizontal)
        self.horizontalSlider_ch2.setTickPosition(QSlider.TicksAbove)
        self.horizontalSlider_ch2.setTickInterval(100)

        self.gridLayout.addWidget(self.horizontalSlider_ch2, 2, 1, 1, 1)

        self.horizontalSlider_ch1 = QSlider(self.gridLayoutWidget)
        self.horizontalSlider_ch1.setObjectName(u"horizontalSlider_ch1")
        self.horizontalSlider_ch1.setMaximum(3000)
        self.horizontalSlider_ch1.setValue(1500)
        self.horizontalSlider_ch1.setOrientation(Qt.Horizontal)
        self.horizontalSlider_ch1.setTickPosition(QSlider.TicksAbove)
        self.horizontalSlider_ch1.setTickInterval(100)

        self.gridLayout.addWidget(self.horizontalSlider_ch1, 0, 1, 1, 1)

        self.label_ch1 = QLabel(self.gridLayoutWidget)
        self.label_ch1.setObjectName(u"label_ch1")
        self.label_ch1.setAlignment(Qt.AlignCenter)

        self.gridLayout.addWidget(self.label_ch1, 0, 0, 1, 1)

        self.horizontalSlider_ch3 = QSlider(self.gridLayoutWidget)
        self.horizontalSlider_ch3.setObjectName(u"horizontalSlider_ch3")
        self.horizontalSlider_ch3.setMaximum(3000)
        self.horizontalSlider_ch3.setValue(1500)
        self.horizontalSlider_ch3.setOrientation(Qt.Horizontal)
        self.horizontalSlider_ch3.setTickPosition(QSlider.TicksAbove)
        self.horizontalSlider_ch3.setTickInterval(100)

        self.gridLayout.addWidget(self.horizontalSlider_ch3, 1, 1, 1, 1)

        self.label_ch1_value = QLabel(self.gridLayoutWidget)
        self.label_ch1_value.setObjectName(u"label_ch1_value")
        self.label_ch1_value.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout.addWidget(self.label_ch1_value, 0, 2, 1, 1)

        self.label_ch2_value = QLabel(self.gridLayoutWidget)
        self.label_ch2_value.setObjectName(u"label_ch2_value")
        self.label_ch2_value.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout.addWidget(self.label_ch2_value, 1, 2, 1, 1)

        self.label_ch3_value = QLabel(self.gridLayoutWidget)
        self.label_ch3_value.setObjectName(u"label_ch3_value")
        self.label_ch3_value.setAlignment(Qt.AlignRight|Qt.AlignTrailing|Qt.AlignVCenter)

        self.gridLayout.addWidget(self.label_ch3_value, 2, 2, 1, 1)

        self.pushButton_reset = QPushButton(self.gridLayoutWidget)
        self.pushButton_reset.setObjectName(u"pushButton_reset")

        self.gridLayout.addWidget(self.pushButton_reset, 3, 0, 1, 3)

        QWidget.setTabOrder(self.horizontalSlider_ch1, self.horizontalSlider_ch3)
        QWidget.setTabOrder(self.horizontalSlider_ch3, self.horizontalSlider_ch2)

        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.label_ch3.setText(QCoreApplication.translate("Form", u"Ch2", None))
        self.label_ch2.setText(QCoreApplication.translate("Form", u"Ch3", None))
        self.label_ch1.setText(QCoreApplication.translate("Form", u"Ch1", None))
        self.label_ch1_value.setText(QCoreApplication.translate("Form", u"1500", None))
        self.label_ch2_value.setText(QCoreApplication.translate("Form", u"1500", None))
        self.label_ch3_value.setText(QCoreApplication.translate("Form", u"1500", None))
        self.pushButton_reset.setText(QCoreApplication.translate("Form", u"Reset", None))
    # retranslateUi

