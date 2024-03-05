#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
from time import time

class Int8MultiArrayPublisher(Node):
    def __init__(self):
        super().__init__('custom_int8multiarray_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'speed_steer_brake_test_topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.msg = Float32MultiArray()
        
        
        self.index_vehile_speed = 5*0.277   # 车速 5*0.277 5km/h
        self.index_steer = 0                # 转向值
        self.index_braek = 0                # 刹车值
        self.index_steer_mode = 2           # 转向模式 1 异向模式
        self.index_dring_mode = 1           # 动力模式 0 踏板模式 1 速度模式
        self.index_gear = 4                 # 档位 4 D
          
    
    def timer_callback(self):
        self.msg.data = [self.index_vehile_speed, self.index_steer, self.index_braek, 
                         self.index_steer_mode, self.index_dring_mode, self.index_gear] 
        self.logical_processing()
        self.get_logger().info('Publishing: {}'.format(self.msg.data))
        self.publisher_.publish(self.msg)

#------------------------------------------------------------------------------------------------------
    # 请在这里实现逻辑变换--这个函数1s执行一次
    def logical_processing(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        elapsed = current_time - self.start_time
        self.index_steer = abs(500 * math.sin(elapsed))  # 转向值 
        self.index_braek = 10 if elapsed % 4 == 0 else 0  # 刹车值
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
