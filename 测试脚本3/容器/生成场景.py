#!/usr/bin/env python3
"""同一份几何同时生成占据地图和 Gazebo 碰撞实体，避免只有可视化范围。"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

CORRIDOR = [(15.0, 18.5), (45.0, 18.5), (45.0, 21.5), (15.0, 21.5)]
START = (43.5, 19.75, 3.141592653589793)
TRIGGER_X = 30.5
FINISH = (16.5, 19.75, 3.141592653589793)
SLOT = (30.0, 21.55, 36.0, 24.8)
# name, xmin, ymin, xmax, ymax, height。上侧两辆停放车辆，下侧和上侧为墙。
OBSTACLES = [
    ("parked_left", 25.5, 21.55, 30.0, 24.2, 1.5),
    ("parked_right", 36.0, 21.55, 40.5, 24.2, 1.5),
    ("south_wall", 14.0, 16.8, 46.0, 17.5, 1.2),
    ("north_wall", 14.0, 24.8, 46.0, 25.0, 1.2),
]
BOUNDS = (10.0, 14.0, 50.0, 28.0)
RESOLUTION = 0.05


def generate(directory, params=None, rviz_template=None):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    x0, y0, x1, y1 = BOUNDS
    width, height = round((x1 - x0) / RESOLUTION), round((y1 - y0) / RESOLUTION)
    pixels = bytearray([254]) * (width * height)
    for _, ax, ay, bx, by, _ in OBSTACLES:
        for iy in range(round((ay - y0) / RESOLUTION), round((by - y0) / RESOLUTION)):
            left, right = round((ax - x0) / RESOLUTION), round((bx - x0) / RESOLUTION)
            begin = (height - 1 - iy) * width + left
            pixels[begin:begin + right - left] = bytes(right - left)
    (directory / "scene.pgm").write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
    (directory / "scene.yaml").write_text(
        f"image: scene.pgm\nmode: trinary\nresolution: {RESOLUTION}\n"
        f"origin: [{x0}, {y0}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")

    sdf = ET.Element("sdf", version="1.7")
    world = ET.SubElement(sdf, "world", name="default")
    ET.SubElement(world, "plugin", name="gazebo_ros_state", filename="libgazebo_ros_state.so")
    physics = ET.SubElement(world, "physics", type="ode")
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_update_rate").text = "1000"
    ET.SubElement(physics, "real_time_factor").text = "1"
    ground = ET.SubElement(world, "model", name="ground_plane")
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", name="ground")
    for kind in ("collision", "visual"):
        plane = ET.SubElement(ET.SubElement(ET.SubElement(link, kind, name=kind), "geometry"), "plane")
        ET.SubElement(plane, "normal").text = "0 0 1"
        ET.SubElement(plane, "size").text = "200 200"
    sun = ET.SubElement(world, "light", name="sun", type="directional")
    ET.SubElement(sun, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(sun, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(sun, "direction").text = "-0.5 0.1 -0.9"
    for name, ax, ay, bx, by, z in OBSTACLES:
        model = ET.SubElement(world, "model", name=name)
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{(ax+bx)/2} {(ay+by)/2} {z/2} 0 0 0"
        link = ET.SubElement(model, "link", name="body")
        for kind in ("collision", "visual"):
            shape = ET.SubElement(link, kind, name=kind)
            box = ET.SubElement(ET.SubElement(shape, "geometry"), "box")
            ET.SubElement(box, "size").text = f"{bx-ax} {by-ay} {z}"
            if kind == "visual":
                material = ET.SubElement(shape, "material")
                color = "0.25 0.35 0.65 1" if name.startswith("parked") else "0.4 0.4 0.4 1"
                ET.SubElement(material, "ambient").text = color
                ET.SubElement(material, "diffuse").text = color
    ET.indent(sdf)
    ET.ElementTree(sdf).write(directory / "scene.world", encoding="utf-8", xml_declaration=True)
    (directory / "geometry.json").write_text(json.dumps({
        "corridor": CORRIDOR, "start": START, "trigger_x": TRIGGER_X,
        "finish": FINISH, "slot": SLOT, "obstacles": OBSTACLES,
        "resolution": RESOLUTION, "bounds": BOUNDS,
    }, ensure_ascii=False, indent=2))

    if params:
        import yaml
        config = yaml.safe_load(Path(params).read_text())
        for name in ("global_costmap", "local_costmap"):
            costmap = config[name][name]["ros__parameters"]
            assert "static_layer" in costmap["plugins"], "必须启用 StaticLayer"
            costmap["always_send_full_costmap"] = True
            costmap["resolution"] = RESOLUTION
            # ROS2 参数解析器会把 YAML 空数组视为未设置；省略后使用节点的默认空过滤器。
            costmap.pop("filters", None)  # 本场景由静态地图提供实体障碍，不依赖外部 keepout mask。
        config["controller_server"]["ros__parameters"]["FollowPath"]["max_vel_x"] = 1.0
        config["velocity_smoother"]["ros__parameters"]["max_velocity"][0] = 1.0
        config["collision_monitor"]["ros__parameters"]["cmd_vel_out_topic"] = "cmd_vel"
        (directory / "nav_params.yaml").write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    if rviz_template:
        import yaml
        config = yaml.safe_load(Path(rviz_template).read_text())
        manager = config["Visualization Manager"]
        manager["Views"]["Current"].update({"X": 30.0, "Y": 21.0, "Scale": 29.0, "Angle": 0.0})
        for display in manager["Displays"]:
            if display.get("Class") == "rviz_default_plugins/Map":
                display["Enabled"] = True
        (directory / "view.rviz").write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    print(f"已生成地图、Gazebo 实体及几何：{directory}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--params")
    parser.add_argument("--rviz-template")
    args = parser.parse_args()
    generate(args.out, args.params, args.rviz_template)
