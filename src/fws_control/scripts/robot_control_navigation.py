#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import math
import numpy as np
import threading
from dynamixel_sdk import *

vel_msg = Twist()

class Commander(Node):

    def __init__(self):
        super().__init__('commander')
        #self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Control table address
        ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
        self.ADDR_MX_GOAL_POSITION = 30
        self.ADDR_GOAL_VELOCITY    = 32
        ADDR_MX_PRESENT_POSITION   = 36
        ADDR_MX_PRESENT_SPEED      = 38

        # Protocol version
        PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

        # Default setting
        DXL_ID                      = 4                 # Dynamixel ID : 1
        BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
        DEVICENAME                  = '/dev/servos'      #ttyUSB0 # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

        TORQUE_ENABLE               = 1                 # Value for enabling the torque
        TORQUE_DISABLE              = 0                 # Value for disabling the torque
        DXL_MINIMUM_POSITION_VALUE  = 0                 # Dynamixel will rotate between this value
        DXL_MAXIMUM_POSITION_VALUE  = 1023              # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
        DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

        index = 0
        dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

        # Enable Dynamixel Torque
        dxl_comm_result1, dxl_error1 = self.packetHandler.write1ByteTxRx(self.portHandler, 1, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result2, dxl_error2 = self.packetHandler.write1ByteTxRx(self.portHandler, 2, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result3, dxl_error3 = self.packetHandler.write1ByteTxRx(self.portHandler, 3, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result4, dxl_error4 = self.packetHandler.write1ByteTxRx(self.portHandler, 4, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result5, dxl_error5 = self.packetHandler.write1ByteTxRx(self.portHandler, 5, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result6, dxl_error6 = self.packetHandler.write1ByteTxRx(self.portHandler, 6, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result7, dxl_error7 = self.packetHandler.write1ByteTxRx(self.portHandler, 7, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        dxl_comm_result8, dxl_error8 = self.packetHandler.write1ByteTxRx(self.portHandler, 8, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
        ''''
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Dynamixel has been successfully connected")
        '''

        self.wheel_seperation = 0.122
        self.wheel_base = 0.156
        self.wheel_radius = 0.026
        self.wheel_steering_y_offset = 0.03
        # For AX-18 mas Speed is 97[rev/min] at12V
        # For AX-12 max Speed is 59[rpm] at 12V
        MAX_RPM = 59
        self.max_rps = MAX_RPM/60
        self.steering_track = self.wheel_seperation - 2*self.wheel_steering_y_offset
        self.mode = 4

        self.GoalVelocities = np.array([0,0,0,0], float) #left_front, right_front, left_rear, right_rear
        self.GoalAngles = np.array([0,0,0,0], float)
        self.vel_servo = np.array([0,0,0,0], int)

    def timer_callback(self):
        global vel_msg

        if(abs(vel_msg.linear.x) > 0.01 and abs(vel_msg.angular.z) > 0.01):
            #node.get_logger.debug("mode1")
            self.mode = 1
            #print("mode1")
            #vel_msg.angular.z = vel_msg.angular.z / 2 #to make car easy to operate
            
            vel_steerring_offset = vel_msg.angular.z * self.wheel_steering_y_offset
            sign = np.sign(vel_msg.linear.x)
            '''
            self.GoalVelocities[0] = -sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) / self.wheel_radius - vel_steerring_offset
            self.GoalVelocities[1] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) / self.wheel_radius + vel_steerring_offset
            self.GoalVelocities[2] = sign*math.hypot(vel_msg.linear.x + vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) / self.wheel_radius + vel_steerring_offset
            self.GoalVelocities[3] = -sign*math.hypot(vel_msg.linear.x - vel_msg.angular.z*self.steering_track/2, vel_msg.angular.z*self.wheel_base/2) / self.wheel_radius - vel_steerring_offset
            '''
            c = 1
            self.GoalVelocities[0] = -sign*math.hypot(vel_msg.linear.x - c*vel_msg.angular.z*self.steering_track/2, c*vel_msg.angular.z*self.wheel_base/2) - c*vel_steerring_offset
            self.GoalVelocities[1] = sign*math.hypot(vel_msg.linear.x + c*vel_msg.angular.z*self.steering_track/2, c*vel_msg.angular.z*self.wheel_base/2) + c*vel_steerring_offset
            self.GoalVelocities[2] = sign*math.hypot(vel_msg.linear.x + c*vel_msg.angular.z*self.steering_track/2, c*vel_msg.angular.z*self.wheel_base/2) + c*vel_steerring_offset
            self.GoalVelocities[3] = -sign*math.hypot(vel_msg.linear.x - c*vel_msg.angular.z*self.steering_track/2, c*vel_msg.angular.z*self.wheel_base/2) - c*vel_steerring_offset

            self.GoalAngles[0] = math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x + vel_msg.angular.z*self.steering_track))*(180/math.pi)
            self.GoalAngles[1] = math.atan(vel_msg.angular.z*self.wheel_base/(2*vel_msg.linear.x - vel_msg.angular.z*self.steering_track))*(180/math.pi)
            self.GoalAngles[2] = -self.GoalAngles[0]
            self.GoalAngles[3] = -self.GoalAngles[1]

        elif(abs(vel_msg.angular.z) < 0.01):
            self.mode = 2
            #print("mode2")
            V = math.hypot(vel_msg.linear.x, vel_msg.linear.y)
            sign = np.sign(vel_msg.linear.x)
            
            if(vel_msg.linear.x != 0):
                ang = vel_msg.linear.y / vel_msg.linear.x
            else:
                ang = 0
            
            self.GoalAngles[0] = -math.atan(ang)*180/math.pi
            self.GoalAngles[1] = -math.atan(ang)*180/math.pi
            self.GoalAngles[2] = self.GoalAngles[0]
            self.GoalAngles[3] = self.GoalAngles[1]
            
            self.GoalVelocities[0] = -sign*V
            self.GoalVelocities[1] = sign*V
            self.GoalVelocities[2] = sign*V
            self.GoalVelocities[3] = -sign*V
            
        elif(abs(vel_msg.linear.x) < 0.01 and abs(vel_msg.linear.y) < 0.01 and abs(vel_msg.angular.z) > 0.01):
            self.mode = 3
            #print("mode3") 

            vel_msg.angular.z = vel_msg.angular.z * 0.5 # too fast rotation will cause self location lost

            self.GoalAngles[0] = math.atan(-self.wheel_base/self.steering_track)*180/math.pi
            self.GoalAngles[1] = math.atan(self.wheel_base/self.steering_track)*180/math.pi
            self.GoalAngles[2] = -math.atan(self.wheel_base/self.steering_track)*180/math.pi
            self.GoalAngles[3] = -math.atan(-self.wheel_base/self.steering_track)*180/math.pi
            
            self.GoalVelocities[0] = vel_msg.angular.z * 0.226
            self.GoalVelocities[1] = vel_msg.angular.z * 0.226
            self.GoalVelocities[2] = self.GoalVelocities[0]
            self.GoalVelocities[3] = self.GoalVelocities[1]
        else:
            self.mode = 4
            #print("mode4")
            self.GoalAngles[0] = 0
            self.GoalAngles[1] = 0
            self.GoalAngles[2] = 0
            self.GoalAngles[3] = 0
            
            self.GoalVelocities[0] = 0
            self.GoalVelocities[1] = 0
            self.GoalVelocities[2] = 0
            self.GoalVelocities[3] = 0       

        #print(f"angles : {self.GoalAngles}")
        #print(f"GoalVelocities : {self.GoalVelocities}")

        #self.GoalAngles = int((self.GoalAngles/150 + 1)*512) # convert to 0~1024

        # convert to 0~1023 if velocity is positive adn to 1024 ~ 2047 if velocity is negative
        if self.mode == 1:
            for i in range(len(self.GoalVelocities)):
                if(self.GoalVelocities[i] >= 0):
                    self.vel_servo[i] = 0.85*self.GoalVelocities[i]/(2*math.pi*self.wheel_radius*self.max_rps)*1023
                else:
                    self.vel_servo[i] = -0.85*self.GoalVelocities[i]/(2*math.pi*self.wheel_radius*self.max_rps)*1023 + 1024
        else:
            for i in range(len(self.GoalVelocities)):
                if(self.GoalVelocities[i] >= 0):
                    self.vel_servo[i] = self.GoalVelocities[i]/(2*math.pi*self.wheel_radius*self.max_rps)*1023
                else:
                    self.vel_servo[i] = -self.GoalVelocities[i]/(2*math.pi*self.wheel_radius*self.max_rps)*1023 + 1024

        for i in range(len(self.vel_servo)):
            if(self.vel_servo[i] > 2047):
                self.vel_servo[i] = 2047
            elif(self.vel_servo[i] < 0):
                self.vel_servo[i] = 0

        #print(f"vel_servo : {self.vel_servo}")

        dxl_comm_result1, dxl_error1 = self.packetHandler.write2ByteTxRx(self.portHandler, 1, self.ADDR_MX_GOAL_POSITION, int(512*(self.GoalAngles[0]/150 +1)))
        dxl_comm_result2, dxl_error2 = self.packetHandler.write2ByteTxRx(self.portHandler, 2, self.ADDR_MX_GOAL_POSITION, int(512*(self.GoalAngles[1]/150 +1)))
        dxl_comm_result3, dxl_error3 = self.packetHandler.write2ByteTxRx(self.portHandler, 3, self.ADDR_MX_GOAL_POSITION, int(512*(self.GoalAngles[2]/150 +1)))
        dxl_comm_result4, dxl_error4 = self.packetHandler.write2ByteTxRx(self.portHandler, 4, self.ADDR_MX_GOAL_POSITION, int(512*(self.GoalAngles[3]/150 +1)))
        dxl_comm_result5, dxl_error5 = self.packetHandler.write2ByteTxRx(self.portHandler, 5, self.ADDR_GOAL_VELOCITY, self.vel_servo[0])
        dxl_comm_result6, dxl_error6 = self.packetHandler.write2ByteTxRx(self.portHandler, 6, self.ADDR_GOAL_VELOCITY, self.vel_servo[1])
        dxl_comm_result7, dxl_error7 = self.packetHandler.write2ByteTxRx(self.portHandler, 7, self.ADDR_GOAL_VELOCITY, self.vel_servo[2])
        dxl_comm_result8, dxl_error8 = self.packetHandler.write2ByteTxRx(self.portHandler, 8, self.ADDR_GOAL_VELOCITY, self.vel_servo[3])

        #self.publisher_.publish(vel_msg)

class Nav_subscriber(Node):

    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_nav',
            self.listener_callback,
            10)
        self.subscription
        self.MAX_RPM = 59

    def listener_callback(self, data):
        global vel_msg
        vel_msg.linear.x = data.linear.x*self.MAX_RPM*math.pi*0.026/30 #AX-18A max_rpm:97 wheel radius:0.026m
        vel_msg.linear.y = data.linear.y*self.MAX_RPM*math.pi*0.026/30
        # the diameter of the radisu (diagonal distance between two wheels) = 0.166 + 0.03*2
        vel_msg.angular.z = -data.angular.z*2*math.pi*(self.MAX_RPM/60)*2*math.pi*0.026/(0.226*math.pi)
        #print(vel_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    
    commander = Commander()
    nav_subscriber = Nav_subscriber()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(commander)
    executor.add_node(nav_subscriber)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    rate = commander.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    rclpy.shutdown()
    executor_thread.join()

