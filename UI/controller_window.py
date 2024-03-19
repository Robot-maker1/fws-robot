# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'controller_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(346, 158)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        Form.setMinimumSize(QtCore.QSize(346, 158))
        Form.setMaximumSize(QtCore.QSize(346, 158))
        self.graphicsView_2 = QtWidgets.QGraphicsView(Form)
        self.graphicsView_2.setGeometry(QtCore.QRect(5, 5, 150, 150))
        self.graphicsView_2.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.graphicsView_2.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.graphicsView_2.setObjectName("graphicsView_2")
        self.mode_groupBox = QtWidgets.QGroupBox(Form)
        self.mode_groupBox.setGeometry(QtCore.QRect(160, 8, 171, 121))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.mode_groupBox.setFont(font)
        self.mode_groupBox.setObjectName("mode_groupBox")
        self.In_phase_radioButton = QtWidgets.QRadioButton(self.mode_groupBox)
        self.In_phase_radioButton.setGeometry(QtCore.QRect(10, 30, 101, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.In_phase_radioButton.setFont(font)
        self.In_phase_radioButton.setChecked(True)
        self.In_phase_radioButton.setObjectName("In_phase_radioButton")
        self.opposite_phase_radioButton = QtWidgets.QRadioButton(self.mode_groupBox)
        self.opposite_phase_radioButton.setGeometry(QtCore.QRect(10, 60, 161, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.opposite_phase_radioButton.setFont(font)
        self.opposite_phase_radioButton.setObjectName("opposite_phase_radioButton")
        self.pivot_turn_radioButton = QtWidgets.QRadioButton(self.mode_groupBox)
        self.pivot_turn_radioButton.setGeometry(QtCore.QRect(10, 90, 111, 23))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.pivot_turn_radioButton.setFont(font)
        self.pivot_turn_radioButton.setObjectName("pivot_turn_radioButton")

        self.retranslateUi(Form)
        self.In_phase_radioButton.clicked.connect(Form.In_phase) # type: ignore
        self.opposite_phase_radioButton.clicked.connect(Form.opposite_phase) # type: ignore
        self.pivot_turn_radioButton.clicked.connect(Form.pivot_turn) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "controller"))
        self.mode_groupBox.setTitle(_translate("Form", "Mode"))
        self.In_phase_radioButton.setText(_translate("Form", "In-phase"))
        self.opposite_phase_radioButton.setText(_translate("Form", "Opposite phase"))
        self.pivot_turn_radioButton.setText(_translate("Form", "Pivot turn"))