"""相機像素 <-> 地面點 的幾何(ENU 世界座標、機身 FLU、相機 X前 Y左 Z上、影像 u右 v下)"""
import math, numpy as np

def quat_to_R(qx, qy, qz, qw):
    return np.array([[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                     [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                     [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])

def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def cam_R(Rbody, yaw, tilt_down, stabilized):
    """相機姿態:stabilized=True 時只用機頭方向(雲台穩定),否則用機身完整姿態"""
    base = Rz(yaw) if stabilized else Rbody
    return base @ Ry(tilt_down)

def pixel_to_ground(u, v, fx, cx, cy, p_cam, Rc):
    d = Rc @ np.array([fx, -(u - cx), -(v - cy)], float)
    if d[2] >= -1e-6: return None                       # 光線沒有往下,打不到地面
    s = -p_cam[2] / d[2]
    return p_cam[:2] + s * d[:2]

def ground_to_pixel(g, fx, cx, cy, p_cam, Rc):          # 只用來測試
    d = Rc.T @ (np.array([g[0], g[1], 0.0]) - p_cam)
    return cx - fx * d[1] / d[0], cy - fx * d[2] / d[0]

if __name__ == '__main__':
    rng = np.random.default_rng(1); err = []
    for _ in range(2000):
        yaw = rng.uniform(-math.pi, math.pi); roll, pitch = rng.normal(0, 0.12, 2)
        q = np.array([math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2),
                      math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2),
                      math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2),
                      math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)])
        Rb = quat_to_R(*q)
        assert abs(math.atan2(Rb[1, 0], Rb[0, 0]) - yaw) < 1e-6 or abs(abs(math.atan2(Rb[1, 0], Rb[0, 0]) - yaw) - 2 * math.pi) < 1e-6
        for stab in (True, False):
            Rc = cam_R(Rb, yaw, math.radians(12), stab)
            pc = np.array([rng.uniform(-5, 5), rng.uniform(-5, 5), 2.0])
            fwd = Rc @ np.array([1, 0, 0.]); g = pc[:2] + rng.uniform(2.5, 8) * fwd[:2] / np.linalg.norm(fwd[:2])
            g = g + rng.normal(0, 0.5, 2)
            u, v = ground_to_pixel(g, 381.5, 320, 240, pc, Rc)
            if not (0 <= u < 640 and 0 <= v < 480): continue
            back = pixel_to_ground(u, v, 381.5, 320, 240, pc, Rc)
            err.append(np.linalg.norm(back - g))
    print('來回投影測試 %d 點,最大誤差 %.2e m' % (len(err), max(err)))
    # 傾斜方向檢查:相機往下 12 度時,地平線應在中心上方 fx*tan(12°) 像素
    Rc = cam_R(np.eye(3), 0.0, math.radians(12), True)
    u, v = ground_to_pixel(np.array([1e6, 0.0]), 381.5, 320, 240, np.array([0, 0, 2.0]), Rc)
    print('往下 12°:遠方地平線在第 %.1f 列(應約 %.1f)' % (v, 240 - 381.5 * math.tan(math.radians(12))))
