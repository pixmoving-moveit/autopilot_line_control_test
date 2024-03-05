#!/usr/bin/python
import rclpy
from rclpy.node import Node
from pix_robobus_driver_msgs.msg import SteeringCommand
from pix_robobus_driver_msgs.msg import BrakeCommand
from pix_robobus_driver_msgs.msg import ThrottleCommand
from pix_robobus_driver_msgs.msg import GearCommand
from pix_robobus_driver_msgs.msg import ParkCommand
from pix_robobus_driver_msgs.msg import VehicleModeCommand

from std_msgs.msg import Float32MultiArray

vehivle_speed = 5 * 0.2777 # 5km/h

class SteerBrakeTestPubsher(Node):
    def __init__(self):
        super().__init__('speed_steer_brake_test_node')

        throttle_ctrl_pub = (ThrottleCommand, "/pix_robobus/throttle_command", 10) 
        gear_ctrl_pub = (GearCommand, "/pix_robobus/gear_command", 10) 
        steer_ctrl_pub = (SteeringCommand, "/pix_robobus/steering_command", 10) 
        brake_ctrl_pub = (BrakeCommand, "/pix_robobus/brake_command", 10) 
        park_ctrl_pub = (ParkCommand, "/pix_robobus/park_command", 10) 
        vehicle_ctrl_pub = (VehicleModeCommand, "/pix_robobus/vehicle_mode_command", 10) 
        
        self.throttle_pub = self.create_publisher(*throttle_ctrl_pub)
        self.gear_pub     = self.create_publisher(*gear_ctrl_pub)
        self.steer_pub    = self.create_publisher(*steer_ctrl_pub)
        self.brake_pub    = self.create_publisher(*brake_ctrl_pub)
        self.park_pub     = self.create_publisher(*park_ctrl_pub)
        self.vehicle_pub  = self.create_publisher(*vehicle_ctrl_pub)
        
        self.init_msg()
        
        self.timer = self.create_timer(1/60, self.timer_callback)
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'speed_steer_brake_test_topic',
            self.listener_callback,
            10)
        
        self.gear = 0
        self.driver_mode = 0
        self.steer_mode = 0

    
    def listener_callback(self, msg:Float32MultiArray):
        self.throttle_msg.drive_speed_target = msg.data.index(0)  # vehicle_speed
        self.steer_msg.steer_angle_target = msg.data.index(1)  # steer_number
        self.brake_msg.brake_pedal_target  = msg.data.index(2)  # brake_number
        
        self.steer_mode = msg.data.index(3)
        self.driver_mode = msg.data.index(4)
        self.gear = msg.data.index(5)
    
    def init_msg(self):
        self.throttle_msg = ThrottleCommand()
        # self.throttle_msg.header
        self.throttle_msg.drive_throttle_pedal_target
        self.throttle_msg.drive_speed_target
        self.throttle_msg.drive_en_ctrl = True
        
        self.steer_msg    = SteeringCommand()
        # self.steer_msg.header
        self.steer_msg.steer_angle_speed = 125
        self.steer_msg.steer_angle_target 
        self.steer_msg.steer_en_ctrl = True 
        
        self.brake_msg    = BrakeCommand()
        # self.brake_msg.header
        self.brake_msg.brake_pedal_target  
        self.brake_msg.brake_en_ctrl = True
        
        # ----------------------------------------
        self.gear_msg     = GearCommand()
        # self.gear_msg.header
        self.gear_msg.gear_target = self.steer_mode  # D档
        self.gear_msg.gear_en_ctrl = True
        
        self.park_msg     = ParkCommand()
        # self.park_msg.header
        self.park_msg.park_target = False
        self.park_msg.park_en_ctrl = True
        
        self.vehicle_msg  = VehicleModeCommand()
        # self.vehicle_msg.header
        self.vehicle_msg.steer_mode_ctrl = self.steer_mode   # 异向转向
        self.vehicle_msg.drive_mode_ctrl = self.driver_mode  # 0 踏板模式 1 转向模式
        
    
    def timer_callback(self):
        self.throttle_pub.publish(self.throttle_msg) 
        self.gear_pub.publish(self.gear_msg)     
        self.steer_pub.publish(self.steer_msg)    
        self.brake_pub.publish(self.brake_msg)    
        self.park_pub.publish(self.park_msg)     
        self.vehicle_pub.publish(self.vehicle_msg)  


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = SteerBrakeTestPubsher()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()