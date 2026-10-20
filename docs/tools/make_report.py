"""Create the report using the exact chapter order of the supplied week-4 PDF."""
import json
import re
import shutil
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch
from PIL import Image

root=Path(__file__).resolve().parents[1]
report=root/'report'
results=root/'results'
m=json.loads((results/'metrics.json').read_text())
run=Path(m['run_dir'])
pub=json.loads((root/'publication.json').read_text()) if (root/'publication.json').exists() else {}

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
fig,ax=plt.subplots(figsize=(12,4.2))
ax.set_xlim(0,12);ax.set_ylim(0,4.2);ax.axis('off')

def box(x,y,w,h,text):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.08',facecolor='#f5f7fa',
                               edgecolor='#314456',linewidth=1.1))
    ax.text(x+w/2,y+h/2,text,ha='center',va='center',fontsize=9)

def arrow(a,b,label=''):
    ax.annotate('',xy=b,xytext=a,arrowprops={'arrowstyle':'->','color':'#314456','lw':1.2})
    if label:ax.text((a[0]+b[0])/2,(a[1]+b[1])/2+.13,label,fontsize=8,ha='center')

box(.15,1.5,1.6,1.0,'Gazebo\nTurtleBot 4')
box(2.4,2.5,2.0,.9,'LiDAR / odom / TF\nscan + pose')
box(2.4,.55,2.0,.9,'OAK-D RGB\nImage 320 × 240')
box(5.1,2.5,2.3,.9,'MapServer + AMCL\nmap → odom')
box(5.1,.55,2.3,.9,'cv_bridge → YOLO-World\nclass / conf / box')
box(8.1,2.5,2.0,.9,'Nav2\nplanner / controller')
box(8.1,.55,3.4,.9,'OpenCV + Image / JSON\nTerminal + evidence')
box(8.1,3.75,2.0,.32,'Node gửi Goal 1 → 2 → 3')
arrow((1.75,2.1),(2.4,2.9));arrow((1.75,1.9),(2.4,1.0))
arrow((4.4,2.95),(5.1,2.95));arrow((4.4,1),(5.1,1))
arrow((7.4,2.95),(8.1,2.95),'pose')
arrow((7.4,1),(8.1,1));arrow((9.1,3.75),(9.1,3.4),'action')
ax.annotate('',xy=(.95,1.48),xytext=(9.1,2.45),
            arrowprops={'arrowstyle':'->','connectionstyle':'angle,angleA=-90,angleB=0,rad=12',
                        'color':'#687a89','lw':1.2})
ax.text(6.2,1.72,'/cmd_vel → chuyển động → sensor cập nhật',fontsize=8,ha='center',color='#314456')
fig.savefig(results/'system_flow.pdf',bbox_inches='tight')
fig.savefig(results/'system_flow.png',bbox_inches='tight',dpi=200)
plt.close(fig)

source=root.parent/'maps/map.pgm'
if not source.exists():
    source=Path('/home/admin1/turtlebot5_github_ws/src/bai04_turtlebot4/map.pgm')
Image.open(source).save(results/'map_original.png')

def num(v,digits=2):return f'{v:.{digits}f}'.replace('.',',')

goals=m['goals'];y=m['yolo']
initial=next(e for e in m['events'] if e['event']=='INITIAL_POSE_FROM_GAZEBO')['pose']
best=m['best_detections'].get('person')
shortest=min(goals,key=lambda g:g['amcl_path_m']);longest=max(goals,key=lambda g:g['amcl_path_m'])
video_url=pub.get('drive_url')
video_filename=Path(pub.get('video_file', 'Demo_23020719_Bai_thuc_hanh_05.webm')).name
repo=pub.get('repo_url','https://github.com/vanhhhhh512/turtlebot4_bai05')
video_cover=(r'\href{'+video_url+r'}{Google Drive}') if video_url else r'\textit{WebM kèm hồ sơ; chưa có link Drive}'
video_body=(r'\url{'+video_url+'}') if video_url else r'\code{'+video_filename+r'} trong hồ sơ local; video chưa được upload/chia sẻ trên Drive nên chưa có link công khai'
tokens={
'REPO_URL':repo,'VIDEO_COVER':video_cover,'VIDEO_BODY':video_body,
'RUN_ID':m['run_id'],'CODE_COMMIT':pub.get('code_commit',pub.get('commit',''))[:7],
'GOAL_TARGET_ROWS':'\n'.join(f"{g['goal']} & {num(g['target']['x'],2)} & {num(g['target']['y'],2)} & {num(g['target']['yaw'],4)} & map \\\\" for g in goals),
'AMCL_GT_MAX':num(m['sampled_amcl_gt_xy_max_m'],3),
'OBSTACLE_DISTANCE':num(m['goal3_obstacle']['minimum_center_distance_m'],3),
'GOAL3_CONFIDENCE':num(max(d['confidence'] for d in m['goal3_obstacle']['camera_evidence']['detections'] if d['class_name']=='fire extinguisher'),3),
'INITIAL_POSE':f"x={num(initial['x'],3)}\\,\\mathrm{{m}}; y={num(initial['y'],3)}\\,\\mathrm{{m}}; yaw={num(initial['yaw'],3)}\\,\\mathrm{{rad}}",
'BEST_PERSON':num(best['detection']['confidence'],3) if best else 'không có',
'SUCCESS_COUNT':str(sum(g['result']=='SUCCEEDED' for g in goals)),
'GOAL_ROWS':'\n'.join(f"{g['goal']} & {g['result']} & {num(g['amcl_path_m'])} & {num(g['sim_duration_s'])} & {num(g['wall_duration_s'])} & {num(g['amcl_goal_distance_m'],3)} \\\\" for g in goals),
'TOTAL_PATH':num(sum(g['amcl_path_m'] for g in goals)),
'STAMP_GAP':f"{num(min(g['pose_stamp_gap_s'] for g in goals),3)}--{num(max(g['pose_stamp_gap_s'] for g in goals),3)}",
'HAZARD_MESSAGES':str(goals[-1]['hazard_messages_received']),
'NAV_OBSERVATION':f"Goal {shortest['goal']} có quãng đường ngắn nhất, {num(shortest['amcl_path_m'])} m; Goal {longest['goal']} dài nhất, {num(longest['amcl_path_m'])} m. Thời gian còn phụ thuộc góc rẽ, điều khiển và tốc độ mô phỏng. Em giữ cả số đo thực và biểu đồ để đối chiếu thay vì kết luận chỉ từ ảnh robot ở đích.",
'YOLO_FRAMES':str(y['frames']),'POSITIVE_FRAMES':str(y['positive_frames']),
'PERSON_BOXES':str(y['box_counts_by_class'].get('person',0)),
'EXTINGUISHER_BOXES':str(y['box_counts_by_class'].get('fire extinguisher',0)),
'LATENCY_MEDIAN':num(y['pipeline_ms_median']),'LATENCY_P95':num(y['pipeline_ms_p95']),
'DETECTION_OBSERVATION':f"Log ghi nhận cả person và fire extinguisher từ camera RGB. Confidence person cao nhất là {num(best['detection']['confidence'],3)}, tại giây mô phỏng {num(best['frame']['image_stamp'])}. Bình trên tuyến Goal 3 cũng có ảnh raw và ảnh có box lưu riêng; hình minh họa lấy trực tiếp từ camera của lượt này. Scene chứa một người và hai bình, nhưng các số box là tổng qua nhiều frame.",
}
tex=(report/'report_template.tex').read_text()
for key,val in tokens.items():tex=tex.replace('@'+key+'@',val)
assert not re.findall(r'@[A-Z_]+@',tex), 'Unfilled report placeholders'
for a,b in [('–','--'),('—','---'),('“','``'),('”',"''"),('’',"'")]:tex=tex.replace(a,b)
(report/'Bao_cao_23020719_Bai_thuc_hanh_05.tex').write_text(tex)
(results/'report_provenance.json').write_text(json.dumps({'run_dir':str(run),'metrics_source':str(results/'metrics.json'),
    'reference_pdf':'/home/admin1/23020719_Đỗ_Việt_Anh_week_4.pdf','chapter_count':11,
    'navigation_screenshot':'results/rviz_reference_run04.png (historical interface illustration only)',
    'current_results_run':m['run_id'],'repo_url':repo,'drive_url':video_url,
    'video_file':video_filename,
    'week4_repo_url':pub.get('week4_repo_url'),
    'week4_commit':pub.get('week4_commit'),
    'week5_code_commit':pub.get('code_commit',pub.get('commit')),
    'camera_goal3_evidence':m['goal3_obstacle']['camera_evidence']['saved_stem']},indent=2,ensure_ascii=False))
compiler=shutil.which('tectonic') or str(Path.home()/'.local/bin/tectonic')
subprocess.run([compiler,'-X','compile','--keep-logs',
                'Bao_cao_23020719_Bai_thuc_hanh_05.tex'],cwd=report,check=True)
shutil.copy2(report/'Bao_cao_23020719_Bai_thuc_hanh_05.pdf',results)
print('Report created:', results/'Bao_cao_23020719_Bai_thuc_hanh_05.pdf')
