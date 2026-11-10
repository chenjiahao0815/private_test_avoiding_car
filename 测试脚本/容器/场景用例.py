#!/usr/bin/env python3
"""避车测试用例：通道多边形、机器人行驶中线、来车出现的位置。

只依赖 math，WSL 和容器里都能直接运行：
  python3 场景用例.py --list           列出全部用例
  python3 场景用例.py --check          检查每个用例的几何（来车、停靠点不出测试场）
"""

import math
import sys

HALF_WIDTH = 1.5           # 通道半宽
# nav2 改写参数文件时会把中文路径转义坏，map_server 读不到；一键脚本每次运行前把 测试场/ 复制到这里。
FIELD_DIR = "/tmp/avoid_field"

# map = R(yaw) * gazebo + (x, y)。仿真 /odom 就是 Gazebo 世界坐标，map -> odom 是这个固定变换。
WORLDS = {
    # 容器自带的 corner_world。对齐量是用激光和地图匹配出来的（地图对齐标定.py）。
    "corner_world": {
        "world": None,
        "map": None,
        "map_from_gazebo": (-0.58, 2.94, -0.067),
    },
    # 自建空旷测试场：四周围墙，可用范围 x -4..64，y -4..44，地图和 Gazebo 坐标重合。
    "测试场": {
        "world": FIELD_DIR + "/avoid_field.world",
        "map": FIELD_DIR + "/avoid_field.yaml",
        "map_from_gazebo": (0.0, 0.0, 0.0),
        "bounds": (-4.0, -4.0, 64.0, 44.0),
    },
}


class Path:
    """按弧长参数化的折线。超出两端时沿端点切线外推。"""

    def __init__(self, points):
        cleaned = []
        for p in points:
            if not cleaned or math.hypot(p[0] - cleaned[-1][0], p[1] - cleaned[-1][1]) > 1e-6:
                cleaned.append((float(p[0]), float(p[1])))
        self.points = cleaned
        self.s = [0.0]
        for a, b in zip(cleaned, cleaned[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.length = self.s[-1]

    def _segment(self, s):
        for i in range(1, len(self.s)):
            if s <= self.s[i] or i == len(self.s) - 1:
                return i
        return len(self.s) - 1

    def at(self, s):
        i = 1 if s <= 0.0 else self._segment(s)
        a, b = self.points[i - 1], self.points[i]
        span = self.s[i] - self.s[i - 1]
        heading = math.atan2(b[1] - a[1], b[0] - a[0])
        t = (s - self.s[i - 1]) / span
        if 0.0 < s < self.length:
            t = min(max(t, 0.0), 1.0)
        return a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), heading

    def offset(self, s, lateral):
        """s 处沿左法线偏 lateral 米（负数是右侧）的点和切向。"""
        x, y, heading = self.at(s)
        return x - lateral * math.sin(heading), y + lateral * math.cos(heading), heading

    def project(self, x, y, lo=-1e9, hi=1e9):
        """点在 [lo, hi] 弧长范围内到中线的最近弧长。"""
        best_s, best_d = None, float("inf")
        for i in range(1, len(self.points)):
            s0, s1 = self.s[i - 1], self.s[i]
            if s1 < lo or s0 > hi:
                continue
            a, b = self.points[i - 1], self.points[i]
            dx, dy = b[0] - a[0], b[1] - a[1]
            span2 = dx * dx + dy * dy
            t = 0.0 if span2 < 1e-12 else ((x - a[0]) * dx + (y - a[1]) * dy) / span2
            t = min(max(t, 0.0), 1.0)
            s = min(max(s0 + t * (s1 - s0), lo), hi)
            px, py, _ = self.at(s)
            d = math.hypot(x - px, y - py)
            if d < best_d:
                best_s, best_d = s, d
        return best_s, best_d

    def reversed(self):
        return Path(self.points[::-1])


def densify(points, step=0.25):
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        out.extend((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1))
    return out


def arc(cx, cy, r, a0_deg, a1_deg, step=0.25):
    a0, a1 = math.radians(a0_deg), math.radians(a1_deg)
    n = max(2, int(math.ceil(abs(a1 - a0) * r / step)))
    return [(cx + r * math.cos(a0 + (a1 - a0) * k / n), cy + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def fillet(points, r, step=0.25):
    """把折线每个拐点换成半径 r 的圆弧，给机器人和来车当行驶中线。"""
    out = [points[0]]
    for i in range(1, len(points) - 1):
        p0, v, p1 = points[i - 1], points[i], points[i + 1]
        u1 = _unit(v[0] - p0[0], v[1] - p0[1])
        u2 = _unit(p1[0] - v[0], p1[1] - v[1])
        cross = u1[0] * u2[1] - u1[1] * u2[0]
        turn = math.atan2(cross, u1[0] * u2[0] + u1[1] * u2[1])
        if abs(turn) < 1e-6:
            out.append(v)
            continue
        t = r * math.tan(abs(turn) / 2.0)
        start = (v[0] - u1[0] * t, v[1] - u1[1] * t)
        side = 1.0 if turn > 0 else -1.0
        center = (start[0] - u1[1] * r * side, start[1] + u1[0] * r * side)
        a0 = math.degrees(math.atan2(start[1] - center[1], start[0] - center[0]))
        out.extend(arc(center[0], center[1], r, a0, a0 + math.degrees(turn), step))
    out.append(points[-1])
    return densify(out, step)


def _unit(dx, dy):
    n = math.hypot(dx, dy)
    return dx / n, dy / n


def offset_line(points, d):
    """折线向左偏 d 米，拐点处按斜接，保持直角拐角。"""
    points = Path(points).points
    out = []
    for i, p in enumerate(points):
        if i == 0:
            u = _unit(points[1][0] - p[0], points[1][1] - p[1])
            n = (-u[1], u[0])
            out.append((p[0] + n[0] * d, p[1] + n[1] * d))
        elif i == len(points) - 1:
            u = _unit(p[0] - points[i - 1][0], p[1] - points[i - 1][1])
            n = (-u[1], u[0])
            out.append((p[0] + n[0] * d, p[1] + n[1] * d))
        else:
            u1 = _unit(p[0] - points[i - 1][0], p[1] - points[i - 1][1])
            u2 = _unit(points[i + 1][0] - p[0], points[i + 1][1] - p[1])
            n1, n2 = (-u1[1], u1[0]), (-u2[1], u2[0])
            m = _unit(n1[0] + n2[0], n1[1] + n2[1])
            k = d / (m[0] * n1[0] + m[1] * n1[1])
            out.append((p[0] + m[0] * k, p[1] + m[1] * k))
    return out


def _fork_polygon(stem, left_branch, right_branch, half=HALF_WIDTH):
    """岔路轮廓。left/right 都从岔心出发。沿 stem 前进时 left 在左侧。"""
    right_outer = offset_line(stem + right_branch[1:], -half)
    right_inner = offset_line(right_branch, half)
    left_inner = offset_line(left_branch, -half)
    left_outer = offset_line(stem + left_branch[1:], half)
    polygon = list(right_outer)
    polygon.append(right_inner[-1])
    polygon.extend(reversed(right_inner[:-1]))
    polygon.extend(left_inner[1:])
    polygon.append(left_outer[-1])
    polygon.extend(reversed(left_outer[:-1]))
    return polygon


def tube(points, half=HALF_WIDTH):
    """沿中线两侧各偏 half 米围成的通道多边形。"""
    return offset_line(points, half) + offset_line(points, -half)[::-1]


CASES = {}

# RViz 的 3D 文字没有中文字体，状态牌上用英文标签。
_LABELS = {
    "原地图弧形": "corner-world arc", "直线": "straight", "单弧": "arc", "S形": "S-curve",
    "L形直角": "L square", "L形圆弯": "L round", "U形圆弯": "U round", "U形直角": "U square",
    "T形": "T junction", "折线45度": "zigzag 45", "Z形直角": "Z square", "斜线30度": "diagonal 30",
    "双S": "double S", "S接U": "S then U", "十字路口": "crossroad", "错位路口": "staggered junction",
    "Y形岔路": "Y fork", "环线": "omega loop",
    "左到右": "left->right", "右到左": "right->left", "左进上出": "left in, top out",
    "左下到右上": "down-left -> up-right", "右上到左下": "up-right -> down-left",
    "上进左出": "top in, left out", "下进上出": "bottom in, top out", "上进下出": "top in, bottom out",
}


def label(name):
    return " / ".join(_LABELS.get(part, part) for part in name.split("_"))


def _add(name, title, world, polygon, path, trigger_s, car_ahead=6.5):
    CASES[name] = {
        "name": name,
        "label": label(name),
        "title": title,
        "world": world,
        "polygon": [(round(x, 4), round(y, 4)) for x, y in polygon],
        "path": path,
        "trigger_s": trigger_s,
        "car_ahead": car_ahead,
    }


def _one(name, title, polygon, center, trigger_s, world="测试场"):
    """一个方向就够。复杂形状不再各做正反两遍。"""
    _add(name, title, world, polygon, Path(center), trigger_s)


def flat_s(x0, y0, length, amp, n=90):
    """两端切线水平的 S。amp>0 先上后回，amp<0 先下后回，终点回到 y0。"""
    points = []
    for k in range(n + 1):
        phase = -math.pi / 2.0 + 2.0 * math.pi * k / n
        points.append((x0 + length * k / n, y0 + amp * (math.sin(phase) + 1.0)))
    return points


def _pair(base, title, polygon, center, forward, backward, world="测试场"):
    """同一个通道，正反两个行驶方向。forward/backward 是 (方向名, 来车触发弧长)。"""
    path = Path(center)
    _add(f"{base}_{forward[0]}", f"{title}，{forward[0]}", world, polygon, path, forward[1])
    _add(f"{base}_{backward[0]}", f"{title}，{backward[0]}", world, polygon, path.reversed(), backward[1])


def _build():
    # 原地图 corner_world 上方空地的缓弯通道。下方是六边形障碍，只够从左往右开、停到下侧。
    def corner_center(x):
        return x, -2.1 - 0.45 * math.sin(math.pi * (x - 18.0) / 18.5)
    corner_line = [corner_center(18.0 + 0.25 * k) for k in range(75)]
    _add("原地图弧形_左到右", "原地图 corner_world 缓弯，从左到右", "corner_world",
         tube(corner_line[::2]), Path(corner_line), trigger_s=8.8)

    straight = densify([(15.0, 20.0), (45.0, 20.0)], 0.5)
    _pair("直线", "直线通道（对照）", tube(straight), straight, ("左到右", 14.0), ("右到左", 14.0))

    single_arc = arc(30.0, -2.0, 22.0, 125.0, 55.0)
    _pair("单弧", "单段圆弧（向上拱，半径 22 m）", tube(single_arc), single_arc,
          ("左到右", 13.0), ("右到左", 13.0))

    s_curve = [(15.0 + 0.25 * k, 20.0 + 4.0 * math.sin(2.0 * math.pi * 0.25 * k / 30.0)) for k in range(121)]
    _pair("S形", "S 形弯（振幅 4 m，波长 30 m）", tube(s_curve), s_curve, ("左到右", 18.0), ("右到左", 18.0))

    l_raw = [(14.0, 12.0), (38.0, 12.0), (38.0, 34.0)]
    _pair("L形直角", "L 形直角弯", tube(l_raw), fillet(l_raw, 2.5),
          ("左进上出", 21.0), ("上进左出", 19.0))

    l_round = fillet(l_raw, 6.0)
    _pair("L形圆弯", "L 形圆角弯（半径 6 m）", tube(l_round), l_round,
          ("左进上出", 21.0), ("上进左出", 19.0))

    u_round = densify([(42.0, 14.0), (22.0, 14.0)]) + arc(22.0, 20.0, 6.0, -90.0, -270.0) + \
        densify([(22.0, 26.0), (42.0, 26.0)])
    _pair("U形圆弯", "U 形掉头弯（开口朝右，半径 6 m）", tube(u_round), u_round,
          ("下进上出", 25.0), ("上进下出", 25.0))

    u_raw = [(42.0, 14.0), (18.0, 14.0), (18.0, 26.0), (42.0, 26.0)]
    _pair("U形直角", "U 形直角掉头（开口朝右）", tube(u_raw), fillet(u_raw, 2.5),
          ("下进上出", 27.0), ("上进下出", 27.0))

    h = HALF_WIDTH
    t_polygon = [(15.0, 20.0 - h), (28.5, 20.0 - h), (28.5, 9.0), (31.5, 9.0), (31.5, 20.0 - h),
                 (45.0, 20.0 - h), (45.0, 20.0 + h), (15.0, 20.0 + h)]
    t_bar = densify([(15.0, 20.0), (45.0, 20.0)], 0.5)
    _pair("T形", "T 形路口（支路朝下），沿横路行驶", t_polygon, t_bar, ("左到右", 18.0), ("右到左", 18.0))

    zig_raw = [(14.0, 15.0), (24.0, 15.0), (32.0, 23.0), (46.0, 23.0)]
    _pair("折线45度", "两处 45 度折弯", tube(zig_raw), fillet(zig_raw, 2.5),
          ("左到右", 15.0), ("右到左", 19.0))

    # 两段横路中间一段竖路，比 L 形多一个反向拐角。
    z_raw = [(12.0, 8.0), (30.0, 8.0), (30.0, 22.0), (52.0, 22.0)]
    _pair("Z形直角", "Z 形两处直角弯", tube(z_raw), fillet(z_raw, 2.5),
          ("左到右", 22.0), ("右到左", 22.0))

    diagonal = densify([(10.0, 8.0), (48.0, 30.0)], 0.5)
    _pair("斜线30度", "约 30 度斜向直线", tube(diagonal), diagonal,
          ("左下到右上", 16.0), ("右上到左下", 16.0))

    # 两端切线水平，两段 S 反向相接，曲率在中点换向。
    double_s = flat_s(8.0, 16.0, 22.0, 3.2) + flat_s(30.0, 16.0, 20.0, -2.8)[1:]
    _one("双S_串联", "两段反向 S 相接，前段先上后回，后段先下后回", tube(double_s), double_s, 18.0)

    # S 的末端切线仍是水平，直接接一个向北的 U，再沿上边返回。
    s_then = flat_s(8.0, 10.0, 20.0, 2.6)
    u_turn = arc(28.0, 16.0, 6.0, -90.0, 90.0, step=0.3)
    back = densify([(28.0, 22.0), (12.0, 22.0)], 0.4)
    s_u = s_then + u_turn[1:] + back[1:]
    _one("S接U", "S 形接 U 形掉头，再沿平行车道返回", tube(s_u), s_u, 24.0)

    # 十字：横路贯通，上下都有支路。车沿横路走，来车出现在路口里。
    cross = [
        (10.0, 20.5), (30.0, 20.5), (30.0, 8.0), (33.0, 8.0), (33.0, 20.5),
        (54.0, 20.5), (54.0, 23.5), (33.0, 23.5), (33.0, 36.0), (30.0, 36.0),
        (30.0, 23.5), (10.0, 23.5),
    ]
    cross_bar = densify([(10.0, 22.0), (54.0, 22.0)], 0.5)
    _one("十字路口", "十字路口，沿横路穿过上下支路", cross, cross_bar, 16.0)

    # 两个支路不在同一个桩号，一上一下错开。
    stagger = [
        (10.0, 18.5), (22.0, 18.5), (22.0, 8.0), (25.0, 8.0), (25.0, 18.5),
        (54.0, 18.5), (54.0, 21.5), (40.0, 21.5), (40.0, 34.0), (37.0, 34.0),
        (37.0, 21.5), (10.0, 21.5),
    ]
    stagger_bar = densify([(10.0, 20.0), (54.0, 20.0)], 0.5)
    _one("错位路口", "上下支路错开的双路口，沿横路连续穿过", stagger, stagger_bar, 14.0)

    # 主干向东，在岔心分成左上、右下两支，车走左上支。
    stem = [(10.0, 18.0), (28.0, 18.0)]
    left_branch = [(28.0, 18.0), (50.0, 32.0)]
    right_branch = [(28.0, 18.0), (50.0, 6.0)]
    y_drive = fillet(stem + left_branch[1:], 4.0)
    y_poly = _fork_polygon(stem, left_branch, right_branch)
    _one("Y形岔路", "Y 形岔路，驶入左上支，右下支留空", y_poly, y_drive, 16.0)

    # 先向东，再顺时针绕 270°，改向北驶出。
    hook_in = densify([(8.0, 28.0), (22.0, 28.0)], 0.4)
    hook_loop = arc(22.0, 20.0, 8.0, 90.0, -180.0, step=0.35)
    hook_out = densify([(14.0, 20.0), (14.0, 34.0)], 0.4)
    hook = hook_in + hook_loop[1:] + hook_out[1:]
    _one("环线钩", "驶入后顺时针绕 270 度，再向北驶出", tube(hook), hook, 22.0)


_build()


def _inside(point, polygon):
    x, y = point
    inside = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def check():
    ok = True
    for case in CASES.values():
        path, polygon = case["path"], case["polygon"]
        problems = []
        lane = lambda s: path.offset(s, -(HALF_WIDTH - 0.95))[:2]  # noqa: E731
        for s in (1.5, case["trigger_s"], path.length - 1.5):
            if not _inside(lane(s), polygon):
                problems.append(f"车道点 s={s:.1f} 不在通道内")
        car_s = min(case["trigger_s"] + case["car_ahead"], path.length - 2.5)
        if not _inside(path.offset(car_s, 0.45)[:2], polygon):
            problems.append("来车不在通道内")
        if car_s - case["trigger_s"] < 4.0:
            problems.append("来车离机器人太近")
        bounds = WORLDS[case["world"]].get("bounds")
        if bounds:
            for x, y in polygon:
                if not (bounds[0] + 3 < x < bounds[2] - 3 and bounds[1] + 3 < y < bounds[3] - 3):
                    problems.append("通道离围墙太近")
                    break
        status = "OK " if not problems else "ERR"
        ok = ok and not problems
        print(f"{status} {case['name']:<16} 长 {path.length:5.1f} m  顶点 {len(polygon):3d}  "
              f"触发 s={case['trigger_s']:.1f}  {'；'.join(problems)}")
    return ok


if __name__ == "__main__":
    if "--list" in sys.argv:
        for name, case in CASES.items():
            print(f"{name}\t{case['title']}")
    elif "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    else:
        print(__doc__)
