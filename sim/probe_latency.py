#!/usr/bin/env python3
"""量各 topic 送到時已經多舊(用模擬時間 /clock 比對 header.stamp)。跑 20 秒自動結束。"""
import time, numpy as np, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class Probe(Node):
    def __init__(self):
        super().__init__('latency_probe')
        self.clk = None; self.age = {'image': [], 'walker': [], 'mavros_pose': []}
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', lambda m: setattr(self, 'clk', m.clock.sec + m.clock.nanosec * 1e-9), be)
        self.create_subscription(Image, '/camera/image', lambda m: self.rec('image', m.header.stamp), be)
        self.create_subscription(PoseStamped, '/walker/pose', lambda m: self.rec('walker', m.header.stamp), 10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', lambda m: self.rec('mavros_pose', m.header.stamp), qos_profile_sensor_data)
        self.t0 = time.time(); self.last_img = None; self.gaps = []
    def rec(self, k, st):
        if k == 'image':
            now = time.time()
            if self.last_img is not None: self.gaps.append(now - self.last_img)
            self.last_img = now
        if self.clk is None: return
        s = st.sec + st.nanosec * 1e-9
        self.age[k].append(self.clk - s)

def main():
    rclpy.init(); n = Probe()
    while time.time() - n.t0 < 20: rclpy.spin_once(n, timeout_sec=0.05)
    print('\n資料送到時,相對模擬時間 /clock 已經多舊 (秒;負的=比 /clock 還新,或用的不是模擬時間)')
    for k, v in n.age.items():
        v = np.array(v)
        if len(v): print('  %-12s n=%4d  中位 %+.3f  90%% %+.3f  最大 %+.3f' % (k, len(v), np.median(v), np.percentile(v, 90), v.max()))
        else: print('  %-12s 沒收到' % k)
    g = np.array(n.gaps)
    if len(g): print('  影像到達間隔(牆上時間) 中位 %.3f s  ->  約 %.1f fps   最大間隔 %.3f s' % (np.median(g), 1 / np.median(g), g.max()))
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
