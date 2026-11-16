#!/usr/bin/env python3
"""从本次实测轨迹生成可直接打开的 SVG，不依赖 ROS 或绘图库。"""
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def render(run):
    run = Path(run)
    report = json.loads((run / "上位机/report.json").read_text())
    trace = json.loads((run / "上位机/trajectory.json").read_text())
    geometry = json.loads((run / "运行信息/geometry.json").read_text())
    svg = ET.Element("svg", xmlns="http://www.w3.org/2000/svg", viewBox="0 0 1100 430")
    ET.SubElement(svg, "rect", width="1100", height="430", fill="white")
    def text(x, y, label, color="#263238", size=16):
        el = ET.SubElement(svg, "text", x=str(x), y=str(y), fill=color,
                           style=f"font: {size}px sans-serif")
        el.text = label
    def xy(x, y):
        return 55+(x-14)*30, 365-(y-16.8)*30
    def box(rect, color, alpha="1"):
        ax, ay, bx, by = rect
        x, y = xy(ax, by)
        ET.SubElement(svg, "rect", x=str(x), y=str(y), width=str((bx-ax)*30),
                      height=str((by-ay)*30), fill=color, opacity=alpha)
    text(35, 30, "测试脚本3：实体障碍之间停靠", size=22)
    text(35, 55, report["result"], "#16803a" if report["result"] == "PASS" else "#bf3030")
    box(geometry["slot"], "#4ade80", "0.18")
    for name, ax, ay, bx, by, _ in geometry["obstacles"]:
        box((ax, ay, bx, by), "#51658a" if name.startswith("parked") else "#64748b")
    for phase, color in ((False, "#df7c16"), (True, "#1973be")):
        segments, segment = [], []
        for _, x, y, _, state in trace:
            if ("恢复" in state) == phase:
                segment.append(xy(x, y))
            elif segment:
                segments.append(segment)
                segment = []
        segments.append(segment)
        for points in segments:
            if points:
                ET.SubElement(svg, "polyline", points=" ".join(f"{x:.2f},{y:.2f}" for x,y in points),
                              fill="none", stroke=color, **{"stroke-width": "3"})
    for event in report["events"]:
        if event["step"] in ("定位", "来车", "停靠", "恢复行驶完成"):
            pose = event.get("robot", event.get("pose"))
            x, y = xy(*pose[:2])
            ET.SubElement(svg, "circle", cx=str(x), cy=str(y), r="4", fill="#172554")
            text(x+7, y-9, event["step"], size=14)
    text(35, 407, "蓝灰：代价地图与 Gazebo 实体障碍    绿色：空位    橙线：行驶与停靠    蓝线：恢复导航", size=15)
    ET.ElementTree(svg).write(run / "场景轨迹.svg", encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    render(sys.argv[1])
