#! /usr/bin/python3
# -*- coding: utf-8 -*-

# GUI
import sys, math, threading
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.Qt import *
from controller_window import Ui_Form
from PyQt5 import QtCore, QtGui, QtWidgets
# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Joy
from geometry_msgs.msg import Twist
#from std_msgs.msg import String

cmd_Joy = Joy()
cmd_Joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
cmd_Joy.buttons = [0,0,0,0,0,0,0,0,0,0,0]

mode = 2

class GraphicsScene(QGraphicsScene):
    
    pressed = False

    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, 0, 0, 150, 150, parent = None) 
        self.opt = ""

    def setOption(self, opt):
        self.opt = opt

    def mouseReleaseEvent(self,event):
        global cmd_Joy
        self.pressed = False
        window.repaint(75, 75)
        cmd_Joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    def mousePressEvent(self,event):
        self.pressed = True

    def mouseMoveEvent(self,event):
        global cmd_Joy, mode

        if(self.pressed == True):
            x = event.scenePos().x()
            y = event.scenePos().y()
            if(((x - 75)**2 + (y - 75)**2) < 60**2):
                if((x-75) != 0):
                    theta = math.atan2((75-y),(x-75))
                else:
                    theta = 0
                
                if mode == 1: # opposite phase
                    cmd_Joy.axes = [0.0, (math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.sin(theta), 0.0, -(math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.cos(theta), 0.0, 0.0, 0.0, 0.0]
                elif mode == 2: # in-phase
                    cmd_Joy.axes = [-(math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.cos(theta), (math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.sin(theta), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                elif mode == 3: # pivot turn
                    cmd_Joy.axes = [0.0, 0.0, 0.0, -(math.sqrt((x - 75)**2 + (y - 75)**2)/60)*math.cos(theta)*0.5, 0.0, 0.0, 0.0, 0.0]
                else:
                    cmd_Joy.axes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

                window.repaint(x, y)

class Segway_controller(Node):

    def __init__(self):
        super().__init__('segway_controller')

        self.time_interval = 0.02

        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(self.time_interval, self.timer_callback)

    def timer_callback(self):
        global cmd_Joy

        self.publisher_.publish(cmd_Joy)

class GUI(QDialog):

    first_time = True

    def __init__(self,parent=None):
        # GUI
        super(GUI, self).__init__(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(10)

        self.repaint(75, 75)
 
    def repaint(self, x, y):
        if(self.first_time):
            self.scene = GraphicsScene()
            self.first_time = False
        self.ui.graphicsView_2.setScene(self.scene)
        self.scene.addEllipse(0, 0, 150, 150, QPen(QColor(0,182,110)), QBrush(QColor(0,182,110)))
        self.scene.addEllipse(x - 15, y - 15, 30, 30, QPen(Qt.red), QBrush(Qt.red))

    def update(self):
        pass
            
    def opposite_phase(self):
        global mode
        mode = 1

    def In_phase(self):
        global mode
        mode = 2

    def pivot_turn(self):
        global mode
        mode = 3

rclpy.init(args=None)
segway_controller = Segway_controller()
executor = rclpy.executors.MultiThreadedExecutor()
executor.add_node(segway_controller)
executor_thread = threading.Thread(target=executor.spin, daemon=True)
executor_thread.start()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = GUI()
    window.show()
    sys.exit(app.exec_())
