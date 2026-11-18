import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Circle(Node):
    def __init__(self):
        super().__init__('circle')
        self.pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        self.t = 0.0
        self.create_timer(0.1, self.tick)          # 每秒 10 次
    def tick(self):
        self.t += 0.1
        v, w = 3.0, 0.3                            # 速度 3 m/s,半徑 = v/w = 10 m
        m = Twist()
        m.linear.x = v * math.cos(w * self.t)
        m.linear.y = v * math.sin(w * self.t)
        self.pub.publish(m)

rclpy.init()
rclpy.spin(Circle())
