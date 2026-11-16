#!/usr/bin/env python3
"""沿 mk38 原图的自由区域选通道；不重绘地图或改变其像素、原点。"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import numpy as np
from PIL import Image
import yaml


def generate(out, map_dir, params, rviz_template):
    out, source = Path(out), Path(map_dir)
    out.mkdir(parents=True, exist_ok=True)
    meta = yaml.safe_load((source/'capella_map.yaml').read_text())
    image_path = source/meta['image']
    pixels = np.array(Image.open(image_path).convert('L'))
    h,w = pixels.shape
    res = meta['resolution']; ox,oy,yaw0 = meta['origin']
    assert yaw0 == 0 and meta['negate'] == 0
    def world(u,v): return np.array([ox+(u+0.5)*res, oy+(h-v-0.5)*res])
    # 预览图上的中部纵向道路，在原图上逐像素验证整个矩形。
    guess = world(w*.542, h*.28)
    direction = np.array([-.21,-1.0]); direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]])
    length,width = 26.0,3.4
    along,across=np.meshgrid(np.arange(-length/2,length/2+.05,.1),np.arange(-width/2,width/2+.05,.1))
    selected=None
    for shift in np.arange(-4,4.01,.1):
        center=guess+normal*shift
        xy=center[:,None,None]+direction[:,None,None]*along+normal[:,None,None]*across
        ix=((xy[0]-ox)/res).astype(int); iy=h-1-((xy[1]-oy)/res).astype(int)
        if ((ix>=0)&(ix<w)&(iy>=0)&(iy<h)).all() and (pixels[iy,ix]>255*(1-meta['free_thresh'])).all():
            selected=center; break
    assert selected is not None, '选定区域放不下通道，请检查原图；不会自动重绘地图'
    center=selected
    corners=[(center+direction*a+normal*b).tolist() for a,b in [(-length/2,-width/2),(length/2,-width/2),(length/2,width/2),(-length/2,width/2)]]
    yaw=math.atan2(direction[1],direction[0])
    start=(*list(center-direction*(length/2-3)),yaw)
    finish=(*list(center+direction*(length/2-3)),yaw)
    geometry={'corridor':corners,'start':start,'finish':finish,'map_sha256':hashlib.sha256(image_path.read_bytes()).hexdigest(), 'map_source':str(image_path),'resolution':res}
    (out/'geometry.json').write_text(json.dumps(geometry,indent=2))
    (out/'geometry.env').write_text(f'START_X={start[0]}\nSTART_Y={start[1]}\nSTART_YAW={yaw}\n')
    shutil.copy2(image_path,out/'capella_map.png')
    meta['image']='capella_map.png'
    (out/'scene.yaml').write_text(yaml.safe_dump(meta))
    config=yaml.safe_load(Path(params).read_text())
    for name in ('global_costmap','local_costmap'):
        costmap=config[name][name]['ros__parameters']
        assert 'static_layer' in costmap['plugins']
        costmap['resolution']=res
        costmap.pop('filters',None)
        costmap['always_send_full_costmap']=True
        # 原图完整加载，代价地图用滚动窗口避免每次发送 3300 万栅格。
        if name=='global_costmap':
            costmap.update(rolling_window=True,width=60,height=60,track_unknown_space=True)
    config['controller_server']['ros__parameters']['FollowPath']['max_vel_x']=1.0
    config['velocity_smoother']['ros__parameters']['max_velocity'][0]=1.0
    config['collision_monitor']['ros__parameters']['cmd_vel_out_topic']='cmd_vel'
    (out/'nav_params.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    rviz=yaml.safe_load(Path(rviz_template).read_text())
    rviz['Visualization Manager']['Views']['Current'].update(X=float(center[0]),Y=float(center[1]),Scale=20.0)
    (out/'view.rviz').write_text(yaml.safe_dump(rviz,sort_keys=False))
    # 仅生成测试副本，不改 /capella/share 中的正式启动文件。
    launch_dir=Path('/capella/share/capella_ros_launcher/launch')
    if launch_dir.exists():
        localization=out/'map_only.launch.py'
        shutil.copy2(Path(__file__).with_name('地图服务.launch.py'),localization)
        bringup=(launch_dir/'bringup_launch.py').read_text().replace("'localization_launch.py'",repr(str(localization)))
        (out/'bringup.launch.py').write_text(bringup)
        launch=(launch_dir/'nav2_opt.launch.py').read_text()
        launch=launch.replace("PythonLaunchDescriptionSource([capella_ros_launch_dir, '/launch', '/bringup_launch.py'])",
                              "PythonLaunchDescriptionSource("+repr(str(out/'bringup.launch.py'))+")")
        (out/'nav2_opt.launch.py').write_text(launch)
    print(json.dumps(geometry),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('out','map-dir','params','rviz-template'):p.add_argument('--'+name,required=True)
    a=p.parse_args();generate(a.out,a.map_dir,a.params,a.rviz_template)
