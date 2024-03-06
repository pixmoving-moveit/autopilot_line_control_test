#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
from time import time, sleep

class Int8MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('custom_int8multiarray_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'speed_steer_brake_test_topic', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.msg = Float32MultiArray()
        
        self.index_vehile_speed = 5     # 车速 5*0.277 5km/h
        self.index_steer = 0.0                # 转向值
        self.index_braek = 0.0                # 刹车值
        self.index_steer_mode = 2           # 转向模式 1 异向模式
        self.index_dring_mode = 1           # 动力模式 0 踏板模式 1 速度模式
        self.index_gear = 4                 # 档位 4 D
        self.sign = -1
        self.count = 0
        
    
    def timer_callback(self):
        self.steering_test()
        self.count += 1
        if(self.count == 250):
            self.sign *= -1
            self.count = 0
        

    def steering_test(self):
        self.index_steer = 3 * self.sign  # 转向值 
        self.msg.data = [
            float(self.index_vehile_speed), 
            float(self.index_steer), 
            float(self.index_braek),
            float(self.index_steer_mode), 
            float(self.index_dring_mode), 
            float(self.index_gear)]
        self.publisher_.publish(self.msg)
        
#------------------------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    int8multiarray_publisher = Int8MultiArrayPublisher()
    rclpy.spin(int8multiarray_publisher)
    # 销毁节点并关闭ROS 2客户端库
    int8multiarray_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
