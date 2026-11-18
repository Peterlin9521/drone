#!/usr/bin/env python3
"""依世界檔中 walker 的路徑點 + Gazebo 模擬時間,算出人的真實位置,發布 /walker/pose"""
import sys, os, math, xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped

def load(sdf):
    for a in ET.parse(sdf).getroot().iter('actor'):
        if a.get('name') == 'walker':
            w = []
            for p in a.iter('waypoint'):
                x, y, z, r, pi, yaw = map(float, p.find('pose').text.split())
                w.append((float(p.find('time').text), x, y, yaw))
            return sorted(w)
    raise SystemExit('世界檔裡找不到 walker')

class Truth(Node):
    def __init__(self, sdf):
        super().__init__('walker_truth')
        self.w = load(sdf); self.T = self.w[-1][0]; self.last = -1.0
        self.pub = self.create_publisher(PoseStamped, '/walker/pose', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.get_logger().info('路徑點 %d 個,一圈 %.0f 秒' % (len(self.w), self.T))
    def at(self, t):
        t %= self.T
        for (t0, x0, y0, a0), (t1, x1, y1, a1) in zip(self.w, self.w[1:]):
            if t0 <= t <= t1:
                s = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
                return x0 + s * (x1 - x0), y0 + s * (y1 - y0), a0 + s * (a1 - a0)
        return self.w[-1][1:]
    def on_clock(self, m):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        if t - self.last < 1 / 30: return
        self.last = t
        x, y, a = self.at(t)
        p = PoseStamped(); p.header.stamp = m.clock; p.header.frame_id = 'map'
        p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, 0.0
        p.pose.orientation.z, p.pose.orientation.w = math.sin(a / 2), math.cos(a / 2)
        self.pub.publish(p)

rclpy.init()
rclpy.spin(Truth(os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/sim/follow_world.sdf')))
