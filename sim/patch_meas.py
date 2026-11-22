"""target_cam 另外送出『原始地面點』/target/meas(沒經過 target_cam 的 KF),給 mpc_follow2 做唯一一次濾波"""
import os, sys
p = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/sim/target_cam.py'); s = open(p).read()
reps = [
 ("        self.pub = self.create_publisher(PoseStamped, '/target/pose', 10)\n",
  "        self.pub = self.create_publisher(PoseStamped, '/target/pose', 10)\n"
  "        self.pub_meas = self.create_publisher(PoseStamped, '/target/meas', 10)   # 原始量測\n"),
 ("        upd = sel is not None\n",
  "        upd = sel is not None\n"
  "        if upd:\n"
  "            mm = PoseStamped(); mm.header.stamp = msg.header.stamp; mm.header.frame_id = 'map'\n"
  "            mm.pose.position.x, mm.pose.position.y = float(sel[0][0]), float(sel[0][1]); mm.pose.orientation.w = 1.0\n"
  "            self.pub_meas.publish(mm)\n"),
]
for a, b in reps:
    if b in s: continue
    assert s.count(a) == 1, '找不到: ' + a.strip()[:60]
    s = s.replace(a, b)
open(p, 'w').write(s); print('已更新', p)
