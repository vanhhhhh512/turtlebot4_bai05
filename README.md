# Bài thực hành 05 — TurtleBot 4, Nav2 và YOLO

Đỗ Việt Anh — 23020719 — K68E-RE. Ubuntu 22.04, ROS 2 Humble và Ignition Fortress.

Chương trình dùng lại bản đồ `maze` tuần 4, định vị AMCL, gửi ba goal tuần tự bằng
`nav2_simple_commander` và nhận diện ảnh RGB của OAK-D bằng Ultralytics YOLO11n.
Hai node Python chạy độc lập: navigation tiếp tục hoạt động khi xử lý ảnh.

## Cài đặt

Cần ROS 2 Humble và TurtleBot 4 Simulator đã cấu hình repository apt theo
[hướng dẫn TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html).

```bash
sudo apt update
sudo apt install ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge \
  python3-colcon-common-extensions python3-yaml
python3 -m pip install ultralytics opencv-python
mkdir -p ~/turtlebot5_ws/src
cd ~/turtlebot5_ws/src
git clone https://github.com/vanhhhhh512/turtlebot4_bai05.git bai05_turtlebot4
cd ~/turtlebot5_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select bai05_turtlebot4
source install/setup.bash
```

YOLO tự tải `yolo11n.pt` trong lần chạy đầu nếu weights chưa có. Có thể truyền
`yolo_model:=/đường/dẫn/yolo11n.pt` để dùng weights có sẵn. Cần desktop và OpenGL
để chạy Gazebo/RViz/OpenCV. CUDA được sử dụng nếu có; node tự chuyển CPU khi
`torch.cuda.is_available()` là false; có thể chọn `device:=cpu`.

## Chạy toàn bộ bằng một lệnh

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot5_ws/install/setup.bash
export ROS_DOMAIN_ID=75 ROS_LOCALHOST_ONLY=1 IGN_PARTITION=bai05_isolated
ros2 launch bai05_turtlebot4 demo.launch.py output_dir:=$HOME/bai05_run
```

Launch mở Gazebo, AMCL/Nav2, RViz và hai node. Node navigation chờ clock,
ground truth và dock status; undock bằng action nếu robot đang ở dock; lấy pose
Gazebo làm **ước lượng ban đầu chỉ dành cho mô phỏng**, rồi chờ Nav2 active.
AMCL tiếp tục dùng scan, map và odometry để cập nhật vị trí.
Sau đó node gọi `goToPose()` và chờ `isTaskComplete()` cho từng goal.
Lượt có thất bại, timeout, bumper hoặc BUMP hazard làm dừng nhiệm vụ.

Không dùng teleop hoặc công cụ Nav2 Goal trong lúc chương trình chạy. Có thể quan
sát RViz và cửa sổ `Bai05 - TurtleBot4 RGB + YOLO`. Sau khi đủ ba goal, navigation
node kết thúc; camera/YOLO và mô phỏng vẫn chạy để xem kết quả. Nhấn Ctrl+C ở
terminal launch để dừng toàn bộ phiên.

Các goal được đặt trong frame `map`, yaw tính bằng radian:

| Goal | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| 1 | 3.05 | 2.99 | 0.0 |
| 2 | -2.93 | 2.91 | 1.57079632679 |
| 3 | 3.57 | -3.59 | 0.0 |

Sửa `config/goals.yaml` trước lượt mới nếu thay goal. Bản đồ trong `maps/` giữ
nguyên từ bài 04; `worlds/maze_yolo.sdf` giữ tường của maze và bổ sung mô hình
người/ghế. Các vật thêm vào được LiDAR/costmap quan sát, không ghi lại vào map gốc.

## Chạy riêng từng phần

```bash
# Mô phỏng + AMCL/Nav2 + YOLO, chưa gửi route
ros2 launch bai05_turtlebot4 demo.launch.py run_route:=false output_dir:=$HOME/bai05_run

# Trong terminal khác, cùng các biến ROS_DOMAIN_ID/IGN_PARTITION ở trên
ros2 run bai05_turtlebot4 navigate_goals.py --ros-args -p use_sim_time:=true \
  -p goals_file:=$HOME/turtlebot5_ws/src/bai05_turtlebot4/config/goals.yaml \
  -p output_dir:=$HOME/bai05_run

# Chỉ node YOLO khi đã có camera; cần tắt YOLO trong launch nếu chạy lệnh này
ros2 run bai05_turtlebot4 yolo_detector.py --ros-args -p use_sim_time:=true \
  -p image_topic:=/oakd/rgb/preview/image_raw -p device:=cpu
```

Tuỳ chọn launch: `show_rviz:=false`, `yolo:=false`, `device:=cpu`,
`yolo_model:=...`, `map:=...`, `server_display:=...`.
Nếu GUI chạy trên display ảo nhưng server cần dùng OpenGL của desktop, đặt
`DISPLAY=:75` và `server_display:=:0`; mặc định server dùng cùng DISPLAY với GUI.

## Source và dữ liệu ghi nhận

```text
scripts/navigate_goals.py     BasicNavigator, undock/initialpose, goal results, telemetry
scripts/yolo_detector.py      Image -> cv_bridge -> YOLO -> bounding boxes/log/image
launch/demo.launch.py         Gazebo server/GUI, clock bridge, robot, AMCL, Nav2, RViz
config/goals.yaml             Ba pose mục tiêu
config/nav2.yaml              Tham số Nav2 thực sự dùng (Navfn + DWB, 0.26 m/s)
config/gazebo_gui.config      Camera quan sát mô phỏng
maps/map.yaml, map.pgm        Map tuần 4
models/                      Model người/ghế cùng metadata tác giả
worlds/maze_yolo.sdf          World maze thêm vật thể
```

Thư mục output gồm `goal_results.json`, `navigation_events.jsonl`,
`trajectory.csv`, `detections.jsonl`, `yolo_metadata.json` và các ảnh raw/YOLO.
Trajectory lấy mẫu mỗi 0,2 giây mô phỏng; AMCL có thể giữ cùng một pose giữa
các lần cập nhật. Quãng đường từ các mẫu này là tổng độ dời AMCL, không phải phép
đo chính xác quãng đường bánh xe. Detection log đếm box theo frame, không đếm
số vật thể duy nhất hoặc đo accuracy. `pipeline_ms` gồm cv_bridge và predict;
không gồm annotate/publish/show/save.

## Interface chính

`/scan`, `/map`, `/odom`, `/amcl_pose`, `/tf`, `/tf_static`, `/cmd_vel`,
`/navigate_to_pose`, `/undock`, `/sim_ground_truth_pose`, `/bumper_contact`,
`/hazard_detection`, `/oakd/rgb/preview/image_raw`.

YOLO publish `/bai05/yolo/image` (`sensor_msgs/msg/Image`) và
`/bai05/yolo/detections` (`std_msgs/msg/String` chứa JSON với timestamp, class,
confidence, box xyxy). Mission state ở `/bai05/mission_status`.

## Tài liệu và nguồn mô hình

- [TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)
- [Nav2 Simple Commander](https://docs.nav2.org/commander_api/index.html)
- [Ultralytics predict](https://docs.ultralytics.com/modes/predict/)
- [Fuel MaleVisitorOnPhone v2](https://fuel.gazebosim.org/1.0/OpenRobotics/models/MaleVisitorOnPhone/2)
- [Fuel Chair v3](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair/3)

Mã bài tập do sinh viên thực hiện, Apache-2.0. Model Fuel và world upstream giữ
quyền tác giả/điều khoản nguồn; xem `NOTICE`. Weights YOLO không được đưa vào Git.
