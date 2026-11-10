#!/usr/bin/env python3
"""生成空旷测试场：Gazebo world 和同坐标的栅格地图。四周一圈围墙，可用范围 x -4..64，y -4..44。"""

import os

HERE = os.path.dirname(os.path.abspath(__file__))
RES = 0.05
ORIGIN = (-5.0, -5.0)
SIZE = (70.0, 50.0)
WALLS = (-4.0, -4.0, 64.0, 44.0)
THICK = 0.2
HEIGHT = 1.0


def write_map():
    w, h = int(round(SIZE[0] / RES)), int(round(SIZE[1] / RES))
    rows = []
    for r in range(h):
        y = ORIGIN[1] + (h - 1 - r + 0.5) * RES
        row = bytearray(w)
        for c in range(w):
            x = ORIGIN[0] + (c + 0.5) * RES
            inside = WALLS[0] - THICK / 2 <= x <= WALLS[2] + THICK / 2 and \
                WALLS[1] - THICK / 2 <= y <= WALLS[3] + THICK / 2
            wall = inside and (
                abs(x - WALLS[0]) <= THICK / 2 or abs(x - WALLS[2]) <= THICK / 2 or
                abs(y - WALLS[1]) <= THICK / 2 or abs(y - WALLS[3]) <= THICK / 2)
            row[c] = 0 if wall else (254 if inside else 205)
        rows.append(bytes(row))
    with open(os.path.join(HERE, "avoid_field.pgm"), "wb") as f:
        f.write(f"P5\n# avoid field\n{w} {h}\n255\n".encode())
        f.writelines(rows)
    with open(os.path.join(HERE, "avoid_field.yaml"), "w") as f:
        f.write("image: avoid_field.pgm\nmode: trinary\nresolution: 0.05\n"
                f"origin: [{ORIGIN[0]}, {ORIGIN[1]}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def wall(name, cx, cy, sx, sy):
    return f"""    <model name='{name}'>
      <static>1</static>
      <pose>{cx} {cy} {HEIGHT / 2} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'><geometry><box><size>{sx} {sy} {HEIGHT}</size></box></geometry></collision>
        <visual name='visual'><geometry><box><size>{sx} {sy} {HEIGHT}</size></box></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Wood</name></script></material>
        </visual>
      </link>
    </model>
"""


def write_world():
    x0, y0, x1, y1 = WALLS
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    lx, ly = x1 - x0 + THICK, y1 - y0 + THICK
    walls = (wall("wall_south", cx, y0, lx, THICK) + wall("wall_north", cx, y1, lx, THICK) +
             wall("wall_west", x0, cy, THICK, ly) + wall("wall_east", x1, cy, THICK, ly))
    world = f"""<?xml version='1.0'?>
<sdf version='1.7'>
  <world name='default'>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"/>
    <model name='ground_plane'>
      <static>1</static>
      <pose>{cx} {cy} 0 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu2></ode></friction></surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Grey</name></script></material>
        </visual>
      </link>
    </model>
    <light name='sun' type='directional'>
      <cast_shadows>0</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <scene>
      <shadows>0</shadows>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
    </scene>
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>{cx} {cy} 70 0 1.5698 0</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <physics type='ode'>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
    </physics>
{walls}  </world>
</sdf>
"""
    with open(os.path.join(HERE, "avoid_field.world"), "w") as f:
        f.write(world)


if __name__ == "__main__":
    write_map()
    write_world()
    print("已生成 avoid_field.world / avoid_field.yaml / avoid_field.pgm")
