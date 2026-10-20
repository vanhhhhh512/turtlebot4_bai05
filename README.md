# Bài thực hành 05 — TurtleBot 4, Nav2 và YOLO

Chương trình dùng lại bản đồ `maze` tuần 4, định vị AMCL, gửi ba goal tuần tự bằng
`nav2_simple_commander` và nhận diện ảnh RGB của OAK-D bằng YOLO-Worldv2.
Hai node Python chạy độc lập: navigation tiếp tục hoạt động khi xử lý ảnh.

Source nền là repository [Bài 04](https://github.com/vanhhhhh512/turtlebot4_bai04):
package `bai04_turtlebot4` cung cấp bản đồ và launch localization được Bài 05
gọi trực tiếp.

## Cài đặt

Cần ROS 2 Humble và TurtleBot 4 Simulator đã cấu hình repository apt theo
[hướng dẫn TurtleBot 4](https://turtlebot.github.io/turtlebot4-user-manual/software/simulation.html).

```bash
sudo apt update
sudo apt install xdotool ros-humble-turtlebot4-simulator ros-humble-turtlebot4-navigation \
  ros-humble-nav2-simple-commander ros-humble-cv-bridge \
  python3-colcon-common-extensions python3-yaml
python3 -m pip install ultralytics opencv-python
mkdir -p ~/turtlebot5_github_ws/src
cd ~/turtlebot5_github_ws/src
git clone https://github.com/vanhhhhh512/turtlebot4_bai04.git bai04_turtlebot4
git clone https://github.com/vanhhhhh512/turtlebot4_bai05.git bai05_turtlebot4
cd ~/turtlebot5_github_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths src/bai04_turtlebot4 src/bai04_turtlebot4/upstream src/bai05_turtlebot4 \
  --packages-select irobot_create_nodes irobot_create_control bai04_turtlebot4 bai05_turtlebot4 \
  --allow-overriding irobot_create_nodes irobot_create_control \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Chạy toàn bộ bằng một lệnh

Trong checkout Bài 05, chạy lệnh sau. Script tự build hai package GitHub trong
workspace, tạo thư mục output mới và mở Gazebo, RViz, YOLO.

```bash
cd ~/turtlebot5_github_ws/src/bai05_turtlebot4
./scripts/run_demo.sh
```

RViz hiển thị **map + LiDAR `/scan` + ảnh RGB OAK-D** cùng lúc. Ảnh RGB dùng
topic `/oakd/rgb/preview/image_raw` với QoS Best Effort; cửa sổ YOLO riêng
hiển thị bounding box, class và confidence. Script
khởi động map/AMCL, Nav2 rồi mới mở các cửa sổ; route bắt đầu sau khi giao
diện đã xuất hiện.

Launch mở Gazebo, AMCL/Nav2, RViz và hai node. Node navigation chờ clock,
ground truth và dock status; undock bằng action nếu robot đang ở dock; lấy pose
Gazebo làm **ước lượng ban đầu chỉ dành cho mô phỏng**, rồi chờ Nav2 active.
AMCL tiếp tục dùng scan, map và odometry để cập nhật vị trí.
Sau đó node gọi `goToPose()` và chờ `isTaskComplete()` cho từng goal.
Lượt có thất bại, timeout, bumper hoặc BUMP hazard làm dừng nhiệm vụ.

Không dùng teleop hoặc công cụ Nav2 Goal trong lúc chương trình chạy. Có thể quan
sát RViz và cửa sổ `Bai05 - TurtleBot4 RGB + YOLO`.

Các goal được đặt trong frame `map`, yaw tính bằng radian:

| Goal | x (m) | y (m) | yaw (rad) |
|---|---:|---:|---:|
| 1 | -8.5 | 8.5 | -1.57079632679 |
| 2 | -8.5 | -8.5 | 0.0 |
| 3 | 9.0 | -9.0 | 1.57079632679 |

## Source và dữ liệu ghi nhận

```text
scripts/navigate_goals.py     BasicNavigator, undock/initialpose, goal results, telemetry
scripts/yolo_detector.py      Image -> cv_bridge -> YOLO -> bounding boxes/log/image
launch/demo.launch.py         Gazebo server/GUI, clock bridge, robot, AMCL, Nav2, RViz
config/goals.yaml             Ba pose mục tiêu
config/localization.yaml      AMCL với beam skipping cho vật thể mới
config/nav2.yaml              Tham số Nav2 thực sự dùng (Navfn + DWB, 0.26 m/s)
config/gazebo_gui.config      Camera quan sát mô phỏng
maps/map.yaml, map.pgm        Map tuần 4
models/                      Model người, bình chữa cháy và YOLO-World TorchScript
worlds/maze_yolo.sdf          World maze thêm vật thể
worlds/placements.yaml       Vị trí các vật trong Gazebo
```

Thư mục output gồm `goal_results.json`, `navigation_events.jsonl`,
`trajectory.csv`, `detections.jsonl`, `yolo_metadata.json` và các ảnh raw/YOLO.
Trajectory lấy mẫu mỗi 0,2 giây mô phỏng; AMCL có thể giữ cùng một pose giữa
các lần cập nhật. Quãng đường từ các mẫu này là tổng độ dời AMCL.

## Interface chính

`/scan`, `/map`, `/odom`, `/amcl_pose`, `/tf`, `/tf_static`, `/cmd_vel`,
`/navigate_to_pose`, `/undock`, `/sim_ground_truth_pose`, `/bumper_contact`,
`/hazard_detection`, `/oakd/rgb/preview/image_raw`.

YOLO publish `/bai05/yolo/image` (`sensor_msgs/msg/Image`) và
`/bai05/yolo/detections` (`std_msgs/msg/String` chứa JSON với timestamp, class,
confidence, box xyxy). Mission state ở `/bai05/mission_status`.

## Tài liệu tham khảo

- [TurtleBot 4 Navigator](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/turtlebot4_navigator.html)
- [Nav2 Simple Commander](https://docs.nav2.org/commander_api/index.html)
- [Ultralytics predict](https://docs.ultralytics.com/modes/predict/)
