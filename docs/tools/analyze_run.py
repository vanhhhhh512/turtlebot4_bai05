"""Derive all reported measurements and figures from one recorded week-5 run."""
import csv
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

run=Path(sys.argv[1]).resolve()
root=Path(__file__).resolve().parents[1]
out=root/'results'
out.mkdir(exist_ok=True)
goals=json.loads((run/'goal_results.json').read_text())
rows=list(csv.DictReader((run/'trajectory.csv').open()))
events=[json.loads(s) for s in (run/'navigation_events.jsonl').read_text().splitlines()]
detections=[json.loads(s) for s in (run/'detections.jsonl').read_text().splitlines()]
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.spines.top':False,
                     'axes.spines.right':False,'savefig.dpi':200})
colors=['#1665a9','#e48225','#328350']
mapfile=root.parent/'maps/map.pgm'
if not mapfile.exists():
    mapfile=Path('/home/admin1/turtlebot5_github_ws/src/bai05_turtlebot4/maps/map.pgm')
im=np.array(Image.open(mapfile))
scene=[]
for item in ET.parse(run/'world/maze_yolo.sdf').getroot().findall('.//world/include'):
    pose=[float(v) for v in item.findtext('pose').split()]
    scene.append({'name':item.findtext('name'),'x':pose[0],'y':pose[1]})

def save(fig,name):
    fig.savefig(out/(name+'.pdf'),bbox_inches='tight')
    fig.savefig(out/(name+'.png'),bbox_inches='tight')
    plt.close(fig)

fig,ax=plt.subplots(figsize=(7.6,7.0))
ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=(-10,10.05,-10,10.05))
metrics=[]
for goal,color in zip(goals,colors):
    n=goal['goal']
    data=[r for r in rows if int(r['goal'])==n and r['amcl_x'] and r['amcl_y']]
    x=np.array([float(r['amcl_x']) for r in data]); y=np.array([float(r['amcl_y']) for r in data])
    t=np.array([float(r['sim_time']) for r in data])
    cum=np.r_[0,np.cumsum(np.hypot(np.diff(x),np.diff(y)))]
    goal['amcl_path_m']=float(cum[-1]) if len(cum) else 0
    goal['amcl_samples']=len(set(r['amcl_stamp'] for r in data))
    goal['localization_end_xy_m']=math.hypot(goal['final_amcl']['x']-goal['final_gt']['x'],
                                          goal['final_amcl']['y']-goal['final_gt']['y'])
    goal['pose_stamp_gap_s']=abs(goal['amcl_stamp']-goal['gt_stamp'])
    metrics.append(goal)
    ax.plot(x,y,color=color,lw=1.7,label=f'Goal {n}')
    ax.scatter(goal['target']['x'],goal['target']['y'],color=color,marker='x',s=90,lw=2)
    ax.annotate(f" G{n}",(goal['target']['x'],goal['target']['y']),color=color,weight='bold')
    if len(x): ax.scatter(x[0],y[0],facecolor='white',edgecolor=color,s=30)
ax.set(xlabel='x (m)',ylabel='y (m)',title='Bản đồ tuần 4 và quỹ đạo AMCL tuần 5',aspect='equal')
ax.legend(loc='upper left'); ax.grid(alpha=.12)
save(fig,'route_map')

fig,axes=plt.subplots(1,3,figsize=(12,3.4))
for ax,goal,color in zip(axes,goals,colors):
    data=[r for r in rows if int(r['goal'])==goal['goal'] and r['amcl_x']]
    x=[float(r['amcl_x']) for r in data]; y=[float(r['amcl_y']) for r in data]
    ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=(-10,10.05,-10,10.05))
    ax.plot(x,y,color=color,lw=1.6)
    ax.scatter(x[0],y[0],edgecolor=color,facecolor='white',s=35,label='Bắt đầu')
    ax.scatter(x[-1],y[-1],color=color,marker='s',s=30,label='Kết thúc')
    ax.scatter(goal['target']['x'],goal['target']['y'],marker='x',color='red',s=55,label='Goal')
    ax.set(xlabel='x (m)',ylabel='y (m)',title=f"Goal {goal['goal']}",aspect='equal')
    ax.set_xlim(min(x+[goal['target']['x']])-1,max(x+[goal['target']['x']])+1)
    ax.set_ylim(min(y+[goal['target']['y']])-1,max(y+[goal['target']['y']])+1)
    ax.grid(alpha=.2)
axes[-1].legend(fontsize=7)
fig.tight_layout();save(fig,'trajectories')

fig,axes=plt.subplots(1,3,figsize=(11.5,3.0))
for ax,key,label in zip(axes,['amcl_path_m','sim_duration_s','amcl_goal_distance_m'],
                       ['Quãng đường AMCL (m)','Thời gian mô phỏng (s)','Khoảng cách AMCL cuối–goal (m)']):
    values=[g[key] for g in metrics]
    bars=ax.bar(['Goal 1','Goal 2','Goal 3'],values,color=colors,width=.6)
    ax.set_ylabel(label,fontsize=9)
    ax.bar_label(bars,fmt='%.2f',padding=3,fontsize=9)
    ax.set_ylim(0,max(values)*1.22)
    ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
fig.tight_layout();save(fig,'goal_metrics')

fig,ax=plt.subplots(figsize=(9,3.6))
for g,color in zip(goals,colors):
    data=[r for r in rows if int(r['goal'])==g['goal'] and r['amcl_x']]
    x=np.array([float(r['amcl_x']) for r in data]);y=np.array([float(r['amcl_y']) for r in data])
    t=np.array([float(r['sim_time']) for r in data])
    cum=np.r_[0,np.cumsum(np.hypot(np.diff(x),np.diff(y)))]
    ax.plot(t-t[0],cum,label=f"Goal {g['goal']}",color=color)
ax.set(xlabel='Thời gian mô phỏng từ đầu goal (s)',ylabel='Quãng đường AMCL tích lũy (m)')
ax.legend();ax.grid(alpha=.25);fig.tight_layout();save(fig,'cumulative_distance')

counts=Counter(d['class_name'] for frame in detections for d in frame['detections'])
positive=[frame for frame in detections if frame['detections']]
latencies=[d['pipeline_ms'] for d in detections]
yolo={'frames':len(detections),'positive_frames':len(positive),
      'box_counts_by_class':dict(counts),'pipeline_ms_median':float(np.median(latencies)),
      'pipeline_ms_p95':float(np.percentile(latencies,95)),
      'first_image_stamp':detections[0]['image_stamp'],'last_image_stamp':detections[-1]['image_stamp'],
      'note':'Counts are boxes across processed frames, not unique object counts or detection accuracy.'}
best={}
for frame in detections:
    if not frame.get('saved_stem'):continue
    for det in frame['detections']:
        key=det['class_name']
        if key not in best or det['confidence']>best[key]['detection']['confidence']:
            best[key]={'frame':frame,'detection':det}
for cls,record in best.items():
    import shutil
    for kind in ['raw','yolo']:
        source=run/'detections'/(record['frame']['saved_stem']+'_'+kind+'.jpg')
        slug=re.sub(r'[^a-z0-9]+','_',cls.lower()).strip('_')
        shutil.copy2(source,out/f'best_{slug}_{kind}.jpg')

sampled=[r for r in rows if r['amcl_x'] and r['amcl_y'] and r['gt_x'] and r['gt_y']]
errors=[math.hypot(float(r['amcl_x'])-float(r['gt_x']),
                   float(r['amcl_y'])-float(r['gt_y'])) for r in sampled]
fig,ax=plt.subplots(figsize=(9,2.8))
ax.plot([float(r['sim_time']) for r in sampled],errors,color=colors[0],lw=1)
ax.set(xlabel='Thời gian mô phỏng (s)',ylabel='Chênh lệch XY AMCL–GT (m)')
ax.grid(alpha=.2);fig.tight_layout();save(fig,'localization_difference')

fig,ax=plt.subplots(figsize=(7,6.5))
ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=(-10,10.05,-10,10.05))
for obj in scene:
    is_person=obj['name'].startswith('person')
    ax.scatter(obj['x'],obj['y'],s=65,c='#7b4090' if is_person else '#c52f2f',
               marker='^' if is_person else 'o',edgecolors='white',linewidths=.5)
    label='Người' if is_person else ('Bình 1' if 'goal1' in obj['name'] else 'Bình 2')
    ax.annotate(f"{label} ({obj['x']:g}; {obj['y']:g})",(obj['x'],obj['y']),
                xytext=(7,6),textcoords='offset points',fontsize=8)
for goal,color in zip(goals,colors):
    ax.scatter(goal['target']['x'],goal['target']['y'],marker='x',c=color,s=70)
    ax.annotate(f"G{goal['goal']}",(goal['target']['x'],goal['target']['y']),
                xytext=(5,-12),textcoords='offset points',color=color,weight='bold')
ax.set(xlabel='x (m)',ylabel='y (m)',aspect='equal',title='Bố trí scene hiện tại trên map tuần 4')
fig.tight_layout();save(fig,'scene_layout')

obstacle=next(o for o in scene if o['name']=='extinguisher_goal3')
gt3=[r for r in rows if int(r['goal'])==3 and r['gt_x'] and r['gt_y']]
nearest=min(gt3,key=lambda r:math.hypot(float(r['gt_x'])-obstacle['x'],float(r['gt_y'])-obstacle['y']))
distance=math.hypot(float(nearest['gt_x'])-obstacle['x'],float(nearest['gt_y'])-obstacle['y'])
fig,ax=plt.subplots(figsize=(7.5,4.4))
ax.imshow(im,cmap='gray',vmin=0,vmax=255,extent=(-10,10.05,-10,10.05))
ax.plot([float(r['gt_x']) for r in gt3],[float(r['gt_y']) for r in gt3],color=colors[2],lw=2,label='Quỹ đạo GT Goal 3')
ax.add_patch(plt.Circle((obstacle['x'],obstacle['y']),.11,color='#c52f2f',label='Collision bình (r = 0,11 m)'))
ax.plot([obstacle['x'],float(nearest['gt_x'])],[obstacle['y'],float(nearest['gt_y'])],
        '--',color='#444',lw=1,label=f'Khoảng cách hai tâm: {distance:.3f} m')
ax.scatter(float(nearest['gt_x']),float(nearest['gt_y']),s=25,color=colors[2])
ax.set(xlim=(obstacle['x']-2.7,obstacle['x']+3),ylim=(obstacle['y']-2.3,obstacle['y']+2.3),
       xlabel='x (m)',ylabel='y (m)',aspect='equal')
ax.legend(fontsize=8);ax.grid(alpha=.2);fig.tight_layout();save(fig,'obstacle_avoidance')

g3_boxes=[(d['confidence'],frame,d) for frame in detections if frame.get('saved_stem')
          and frame.get('mission')=='Navigating to Goal 3' for d in frame['detections']
          if d['class_name']=='fire extinguisher']
g3_detection=max(g3_boxes,key=lambda item:item[0]) if g3_boxes else None
if g3_detection:
    for kind in ['raw','yolo']:
        shutil.copy2(run/'detections'/(g3_detection[1]['saved_stem']+'_'+kind+'.jpg'),
                     out/f'extinguisher_goal3_{kind}.jpg')

summary={'run_dir':str(run),'run_id':run.name,'goals':metrics,'yolo':yolo,'best_detections':best,
         'events':events,'map_sha256':hashlib.sha256(mapfile.read_bytes()).hexdigest(),
         'scene':scene,'sampled_amcl_gt_xy_max_m':max(errors),
         'amcl_gt_comparison_note':'XY differences use poses stored in one telemetry row; AMCL and Gazebo timestamps may differ. This is not a synchronized independent localization accuracy estimate.',
         'goal3_obstacle':{'position':obstacle,'minimum_center_distance_m':distance,
                           'sim_time':float(nearest['sim_time']),'gt_x':float(nearest['gt_x']),
                           'gt_y':float(nearest['gt_y']),
                           'note':'Distance between robot pose center and extinguisher center, not surface clearance.',
                           'camera_evidence':g3_detection[1] if g3_detection else None},
         'direct_goal_sequence':not any(e['event'].startswith('OBSERVATION') for e in events),
         'measurement_note':'AMCL sampled every 0.2 simulation seconds; duration includes 0.5 wall-second settling after action result. Final AMCL and GT have the timestamp gap reported for each goal.'}
(out/'metrics.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
print(json.dumps({'goals':metrics,'yolo':yolo},ensure_ascii=False,indent=2))
