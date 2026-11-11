#!/usr/bin/env python3
"""双侧避车测试用例：只用矩形通道，靠禁扫区堵住近侧，再用 /car_information_all 控制远侧有没有动态车。

只依赖 math，WSL 和容器里都能直接运行：
  python3 场景用例.py --list           列出全部用例
  python3 场景用例.py --check          检查每个用例的几何
"""

import math
import sys

HALF_WIDTH = 1.5           # 通道半宽
# 机器人离右侧长边的距离。无曲线版通过性允许车身出通道 1 m，靠右 0.95 m 时车能挤过去，机器人不算挡路。
ROBOT_LANE = 1.25
# nav2 改写参数文件时会把中文路径转义坏，map_server 读不到；一键脚本每次运行前把 测试场/ 复制到这里。
FIELD_DIR = "/tmp/avoid_field"

# map = R(yaw) * gazebo + (x, y)。自建测试场的地图和 Gazebo 坐标重合。
WORLDS = {
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


CASES = {}

# 矩形通道：x 15..45，y 18.5..21.5。机器人从左往右开，右手（南边 y=18.5）是近侧长边。
X0, X1, YC = 15.0, 45.0, 20.0
RECT = [(X0, YC - HALF_WIDTH), (X1, YC - HALF_WIDTH), (X1, YC + HALF_WIDTH), (X0, YC + HALF_WIDTH)]
CENTER = Path([(X0, YC), (X1, YC)])


def _band(x0, x1, south):
    """长边外侧的禁扫带：south=True 盖南边（近侧），否则盖北边（远侧）。不碰通道本身。"""
    if south:
        y0, y1 = YC - HALF_WIDTH - 3.6, YC - HALF_WIDTH - 0.05
    else:
        y0, y1 = YC + HALF_WIDTH + 0.05, YC + HALF_WIDTH + 3.6
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


# 找点外侧偏移 1.1~1.4 m 加车身，整条带子都盖住。两头伸出短边 2 m：
# 靠右掉头后机器人往通道起点开、向前找点，候选点能贴着短边落在 x=15 附近。
NO_GO = {
    "near": _band(X0 - 2.0, X1 + 2.0, True),
    "far": _band(X0 - 2.0, X1 + 2.0, False),
    # 只盖机器人身后 4.5 m 以内：原来 3~3.3 m 的范围没点，加大到 2 倍后才有。
    "near_close": _band(25.0, X1 - 1.0, True),
}

# timeline：(触发, 延时秒, 动作, 值)。
#   触发 avoid 发出避车请求；point 第一次收到停靠点标记；right_edge 第一次收到靠右路径
#   动作 dynamic 切 /car_information_all（none 空、busy 一直有车、flicker 有 2 秒空 2 秒、silent 不发）
#        no_go   换禁扫区，值是 NO_GO 里的名字列表
# expect：
#   park         停靠位置相对行驶方向：right / left / none（不停，走完靠右）/ any
#   approve_side 最后批准的 side：1 近侧，2 远侧，None 不批准，"any" 不查
#   right_edge   是否必须走过靠右
#   give_up      远侧放弃原因：busy 连续 4 秒有车，timeout 15 秒，None 不应放弃
#   far_wait     是否必须进入远侧等待（False 表示绝不能进入）
#   find_log     找点日志里必须出现的文字
#   min_wait     远侧点从开始等待到批准至少多少秒
RELEASE = [("right_edge", 1.0, "dynamic", "none")]


def _case(name, title, label, no_go=(), timeline=(), trigger_s=14.0, dual=True, hug="right", **expect):
    full = {"park": "any", "approve_side": "any", "right_edge": False, "give_up": None,
            "far_wait": None, "find_log": None, "min_wait": 3.0}
    full.update(expect)
    CASES[name] = {
        "name": name,
        "label": "rect / " + label,
        "title": title,
        "world": "测试场",
        "polygon": list(RECT),
        "path": CENTER,
        "trigger_s": trigger_s,
        "car_ahead": 6.5,
        "no_go": list(no_go),
        "timeline": list(timeline),
        "dual": dual,
        "hug": hug,
        "expect": full,
    }


def _build():
    # ---------------- 基础流程
    _case("矩形_近侧停靠", "近侧没堵，直接停近侧", "near side",
          park="right", approve_side=1, far_wait=False)
    _case("矩形_远侧无车", "禁扫区堵住近侧，远侧连续 3 秒无车后停远侧", "far side, clear",
          no_go=["near"], park="left", approve_side=2, far_wait=True)
    _case("矩形_远侧持续有车", "近侧堵住，远侧一直有车，4 秒后靠右，5 秒屏蔽后再找", "far side, busy 4s",
          no_go=["near"], timeline=[("avoid", 0.0, "dynamic", "busy")] + RELEASE,
          park="any", right_edge=True, give_up="busy", far_wait=True)
    _case("矩形_远侧车断续", "近侧堵住，远侧车断断续续，等满 15 秒靠右，之后再找", "far side, flicker 15s",
          no_go=["near"], timeline=[("avoid", 0.0, "dynamic", "flicker")] + RELEASE,
          park="any", right_edge=True, give_up="timeout", far_wait=True)
    # ---------------- 近侧范围
    _case("矩形_近侧加大范围", "近侧 3~4.5 m 被堵，2 倍范围里才有点，仍算近侧直接停", "near, 2x range",
          no_go=["near_close"], park="right", approve_side=1, far_wait=False)
    _case("矩形_近侧框越界裁剪", "机器人离通道起点很近，加大的近侧框越过短边，裁掉越界部分后照样找到",
          "near, crop at short edge", trigger_s=6.0,
          park="right", approve_side=1, far_wait=False, find_log="裁到")
    # ---------------- 两侧都堵
    _case("矩形_两侧都堵", "两侧都被禁扫区堵住，一个点都没有，直接靠右走完", "both blocked",
          no_go=["near", "far"], park="none", approve_side=None, right_edge=True, far_wait=False)
    _case("矩形_两侧都堵后放开", "两侧都堵先靠右往后退，退的过程中北侧放开，重新找到点再停",
          "both blocked, then open", no_go=["near", "far"],
          timeline=[("right_edge", 1.0, "no_go", ["near"])],
          park="left", right_edge=True)
    # ---------------- 远侧等待细节
    _case("矩形_确认前来车", "远侧无车 2 秒时来一辆车 1 秒，3 秒计时清零重来，之后再批准远侧",
          "far, car during wait", no_go=["near"],
          timeline=[("point", 2.0, "dynamic", "busy"), ("point", 3.0, "dynamic", "none")],
          park="left", approve_side=2, far_wait=True, min_wait=5.7)
    _case("矩形_动态车话题断流", "远侧等待时收不到 /car_information_all，3 秒和 4 秒暂停，等满 15 秒靠右",
          "far, topic silent", no_go=["near"],
          timeline=[("avoid", 0.0, "dynamic", "silent")] + RELEASE,
          park="any", right_edge=True, give_up="timeout", far_wait=True)
    _case("矩形_等待中近侧放开", "等待中近侧放开，仍保存原远侧点，无车满 3 秒直接停远侧", "saved far point, near opens",
          no_go=["near"], timeline=[("point", 1.0, "no_go", [])],
          park="left", approve_side=2, far_wait=True)
    # ---------------- 开关
    _case("矩形_双侧关闭对照", "GARAGE_ENABLE_DUAL_SIDE_AVOIDANCE=false：老行为树，近侧堵住也绝不找远侧",
          "dual side off", no_go=["near"], dual=False,
          park="any", right_edge=True, far_wait=False)

    # 贴左边行驶。通道仍是这条矩形：北是行驶方向的左边（近侧），南是右边（远侧）。
    # 找点在身后：近侧大约 3~6.6 m，远侧大约 3~3.3 m。倒车掉头后仍贴着北边往起点开，
    # 所以左边继续是近侧，马上能收；右边仍是远侧，要等动态车，而且倒车跟踪开始 5 秒内不收。
    def left(x0, x1):
        return _band(x0, x1, False)

    def right(x0, x1):
        return _band(x0, x1, True)

    NO_GO["左全"] = left(X0 - 2.0, X1 + 2.0)
    NO_GO["右全"] = right(X0 - 2.0, X1 + 2.0)
    # 盖住触发点身后的近侧搜索框，再往起点留出空档，倒车开过去才能看见。
    NO_GO["左近处"] = left(20.0, X1 + 2.0)
    NO_GO["右近处"] = right(24.0, X1 + 2.0)
    # 车位沿边净空。footprint 本地 (±0.925, ±0.660)，车头沿通道时车身长 1.85 m。
    # 找点只要 footprint 和禁扫区有重叠就丢掉这个点，所以净空至少 1.85 m。
    # 两端再各留约 0.67 m：采样间隔 0.15 m、车头略偏，倒车时搜索框一次能跳过 1 m。
    # 3.2 m 是一个车位。十几米的开口是空地，框子落在哪都行，对不齐车身。
    stall = 3.2
    # 直接找到：机器人在 x≈29.6，远侧框在身后 3.0~3.3 m，点在 x≈26.6。
    stall_now = (26.56 - stall / 2.0, 26.56 + stall / 2.0)
    # 倒车找到：掉头后向前搜 5.0~5.3 m，点在 x≈22.6。这个位置一开始的身后框够不着。
    stall_back = (22.55 - stall / 2.0, 22.55 + stall / 2.0)
    NO_GO["右车位前"] = right(stall_now[1], X1 + 2.0)
    NO_GO["右车位后"] = right(X0 - 2.0, stall_now[0])
    NO_GO["右后车位前"] = right(stall_back[1], X1 + 2.0)
    NO_GO["右后车位后"] = right(X0 - 2.0, stall_back[0])

    def left_case(name, title, label, **kwargs):
        kwargs.setdefault("hug", "left")
        _case(name, title, label, **kwargs)

    left_case("1左贴边_右边禁行_左边找到", "贴左边开，右边整段禁行，直接在左边找到", "hug left, right blocked",
              no_go=["右全"], park="left", approve_side=1, far_wait=False)
    left_case("2左贴边_左边部分禁行_倒车再找到", "右边整段禁行，左边近处也禁，倒车到远处的空档再停左边",
              "hug left, left opens behind", no_go=["右全", "左近处"],
              park="left", approve_side=1, right_edge=True, far_wait=False)
    left_case("3左贴边_右边部分禁行_倒车再找到", "左边整段禁行，右边近处禁、远处空，返回途中找到远侧点，等待后再停右边",
              "hug left, right opens behind", no_go=["左全", "右近处"],
              park="right", approve_side=2, right_edge=True, far_wait=True)
    left_case("4左贴边_右边车位_直接找到", "两边都禁，右边身后留一个车位，前后被占，直接停这个车位",
              "hug left, right stall now", no_go=["左全", "右车位前", "右车位后"],
              park="right", approve_side=2, far_wait=True)
    left_case("4左贴边_右边车位_倒车找到", "右边车位更靠后，近处找不到，返回途中找到并等待后停进车位",
              "hug left, right stall behind", no_go=["左全", "右后车位前", "右后车位后"],
              park="right", approve_side=2, right_edge=True, far_wait=True)


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
        sign = 1.0 if case.get("hug") == "left" else -1.0
        lane = lambda s, sign=sign: path.offset(s, sign * (HALF_WIDTH - ROBOT_LANE))[:2]  # noqa: E731
        for s in (1.5, case["trigger_s"], path.length - 1.5):
            if not _inside(lane(s), polygon):
                problems.append(f"车道点 s={s:.1f} 不在通道内")
        car_s = min(case["trigger_s"] + case["car_ahead"], path.length - 2.5)
        car_sign = -sign
        if not _inside(path.offset(car_s, car_sign * 0.45)[:2], polygon):
            problems.append("来车不在通道内")
        names = list(case["no_go"]) + [n for t in case["timeline"] if t[2] == "no_go" for n in t[3]]
        for n in names:
            if n not in NO_GO:
                problems.append(f"没有禁扫区 {n}")
                continue
            if any(_inside(p, NO_GO[n]) for p in polygon):
                problems.append(f"禁扫区 {n} 盖住了通道顶点")
            if _inside(lane(case["trigger_s"]), NO_GO[n]):
                problems.append(f"禁扫区 {n} 盖住了机器人车道")
        for trigger, _, action, value in case["timeline"]:
            if trigger not in ("avoid", "point", "right_edge") or action not in ("dynamic", "no_go"):
                problems.append(f"时间线写错：{trigger} {action}")
            if action == "dynamic" and value not in ("none", "busy", "flicker", "silent"):
                problems.append(f"没有动态车模式 {value}")
        status = "OK " if not problems else "ERR"
        ok = ok and not problems
        e = case["expect"]
        print(f"{status} {case['name']:<14} 触发 s={case['trigger_s']:<5} 禁扫区 {','.join(case['no_go']) or '-':<9} "
              f"停 {e['park']:<5} side {e['approve_side']!s:<4} 靠右 {e['right_edge']!s:<5} "
              f"放弃 {e['give_up']!s:<7} {'；'.join(problems)}")
    return ok


if __name__ == "__main__":
    if "--list" in sys.argv:
        for name, case in CASES.items():
            print(f"{name}\t{case['title']}")
    elif "--check" in sys.argv:
        sys.exit(0 if check() else 1)
    else:
        print(__doc__)
