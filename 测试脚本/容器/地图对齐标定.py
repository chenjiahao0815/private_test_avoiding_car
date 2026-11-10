import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

MAP = "/capella/share/capella_ros_launcher/map/corner_world"


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_map():
    meta = {}
    for line in open(MAP + ".yaml"):
        if ":" in line:
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    origin = [float(v) for v in meta["origin"].strip("[]").split(",")]
    with open(MAP + ".pgm", "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    occ = img < 100
    return occ, res, origin


rclpy.init()
node = Node("align_map", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
data = {}
node.create_subscription(LaserScan, "/scan", lambda m: data.setdefault("scan", m), qos_profile_sensor_data)
node.create_subscription(Odometry, "/odom", lambda m: data.__setitem__("odom", m), 10)
buf = Buffer()
TransformListener(buf, node)
end = time.monotonic() + 15
laser = None
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.1)
    if "scan" in data and "odom" in data:
        try:
            t = buf.lookup_transform("base_footprint", data["scan"].header.frame_id, rclpy.time.Time()).transform
            laser = (t.translation.x, t.translation.y, yaw_of(t.rotation))
            break
        except Exception:
            pass
assert laser, "没有拿到 scan/odom/TF"
scan = data["scan"]
p = data["odom"].pose.pose
rx, ry, rth = p.position.x, p.position.y, yaw_of(p.orientation)
ang = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
r = np.array(scan.ranges)
ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max * 0.98)
ang, r = ang[ok], r[ok]
lx = laser[0] + np.cos(laser[2] + ang) * r
ly = laser[1] + np.sin(laser[2] + ang) * r
wx = rx + math.cos(rth) * lx - math.sin(rth) * ly
wy = ry + math.sin(rth) * lx + math.cos(rth) * ly
print("odom pose", round(rx, 3), round(ry, 3), round(rth, 3), "points", len(wx), "laser", laser)

occ, res, origin = load_map()
h, w = occ.shape
from scipy.ndimage import distance_transform_edt  # noqa: E402
dist = distance_transform_edt(~occ) * res


def score(dx, dy, dth):
    c, s = math.cos(dth), math.sin(dth)
    mx = c * wx - s * wy + dx
    my = s * wx + c * wy + dy
    col = ((mx - origin[0]) / res).astype(int)
    row = (h - 1 - (my - origin[1]) / res).astype(int)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    d = np.full(len(mx), 1.0)
    d[inside] = np.minimum(dist[row[inside], col[inside]], 1.0)
    return d.mean()


best = (9, 0, 0, 0)
for dth in np.arange(-0.1, 0.1001, 0.005):
    for dx in np.arange(-2.0, 0.6, 0.05):
        for dy in np.arange(3.0, 5.6, 0.05):
            v = score(dx, dy, dth)
            if v < best[0]:
                best = (v, dx, dy, dth)
v, bx, by, bt = best
for dth in np.arange(bt - 0.006, bt + 0.0061, 0.001):
    for dx in np.arange(bx - 0.06, bx + 0.061, 0.01):
        for dy in np.arange(by - 0.06, by + 0.061, 0.01):
            s = score(dx, dy, dth)
            if s < best[0]:
                best = (s, dx, dy, dth)
print("best mean dist %.4f  map = R(%.4f) * gazebo + (%.3f, %.3f)" % (best[0], best[3], best[1], best[2]))
print("old guess score %.4f" % score(-0.73, 4.27, -0.01))


def inliers(dx, dy, dth, tol):
    c, s = math.cos(dth), math.sin(dth)
    mx = c * wx - s * wy + dx
    my = s * wx + c * wy + dy
    col = ((mx - origin[0]) / res).astype(int)
    row = (h - 1 - (my - origin[1]) / res).astype(int)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    d = np.full(len(mx), 9.0)
    d[inside] = dist[row[inside], col[inside]]
    return (d <= tol).mean(), np.median(d)


print("inliers@0.1 %.2f inliers@0.2 %.2f median %.3f" % (*inliers(best[1], best[2], best[3], 0.1)[:1], *inliers(best[1], best[2], best[3], 0.2)))
