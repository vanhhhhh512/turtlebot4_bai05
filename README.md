# Bài thực hành 05 — TurtleBot 4, Nav2 và YOLO

Đỗ Việt Anh — 23020719 — K68E-RE. Ubuntu 22.04, ROS 2 Humble và Ignition Fortress.

Chương trình dùng lại bản đồ `maze` tuần 4, định vị AMCL, gửi ba goal tuần tự bằng
`nav2_simple_commander` và nhận diện ảnh RGB của OAK-D bằng YOLO-Worldv2.
Hai node Python chạy độc lập: navigation tiếp tục hoạt động khi xử lý ảnh.

Source nền là repository [Bài 04](https://github.com/vanhhhhh512/turtlebot4_bai04):
package `bai04_turtlebot4` cung cấp bản đồ và launch localization được Bài 05
gọi trực tiếp. Hai file map trong repository này là bản sao nguyên byte để nộp
hồ sơ. Node gửi goal, node YOLO và world thêm vật thể là phần mở rộng Bài 05.

## Video demo, báo cáo và kết quả mới

- [Video demo (Google Drive)](https://drive.google.com/drive/folders/1GKdy0U0tKGbxUAaGAmAffTPxJa5pT8wy?usp=drive_link)
- [Báo cáo PDF — 19 trang, cập nhật 21/10/2026](docs/report/Bao_cao_23020719_Bai_thuc_hanh_05.pdf)
- [GitHub mã nguồn](https://github.com/vanhhhhh512/turtlebot4_bai05)
- [ZIP LaTeX để mở/sửa trên Prism](docs/report/Bao_cao_23020719_Bai05_LaTeX_Prism.zip)
- [Nguồn LaTeX, biểu đồ, script dựng PDF và evidence run10](docs/README.md)

Lượt `direct_goal_route_run10` đạt **3/3 SUCCEEDED, 0 contact, 0 bump**. Goal 3
được gửi trực tiếp sau Goal 2. Bình tại `(-1.8, -5.5)` nằm trên quỹ đạo cũ;
quỹ đạo robot mới cách tâm bình gần nhất 0.618 m. Camera YOLO ghi 132 box person
và 160 box fire extinguisher qua nhiều frame. Chi tiết và giới hạn phép đo nằm
trong báo cáo cùng `docs/results/metrics.json`.

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

YOLO-World nhận diện bình chữa cháy và người. Scene demo giữ một người ở vị trí an toàn để kiểm tra bbox human; bàn, ghế và người bị lỗi vị trí đã được bỏ khỏi scene.

Model YOLO-World TorchScript trong `models/yolo_world_furniture.torchscript` đi kèm source,
với các prompt `wooden table`, `chair`, `a red cylinder with a white stripe` (hiển thị là fire extinguisher), `person`; demo không
cần tải weights ở lần chạy đầu. Có thể truyền `yolo_model:=/đường/dẫn/model.pt`
để dùng model khác. Cần desktop và OpenGL
để chạy Gazebo/RViz/OpenCV. CUDA được sử dụng nếu có; node tự chuyển CPU khi
`torch.cuda.is_available()` là false; có thể chọn `device:=cpu`.

## Chạy toàn bộ bằng một lệnh

Trong checkout Bài 05, chạy lệnh sau. Script tự build hai package GitHub trong
workspace, tạo thư mục output mới và mở Gazebo, RViz, YOLO. Trên màn hình
1920×1080, RViz nằm bên trái (~66% chiều rộng), Gazebo ở góc trên bên phải và
YOLO ở góc dưới bên phải; màn hình khác được chia cùng tỷ lệ.

```bash
cd ~/turtlebot5_github_ws/src/bai05_turtlebot4
./scripts/run_demo.sh
```

RViz hiển thị **map + LiDAR `/scan` + ảnh RGB OAK-D** cùng lúc. Ảnh RGB dùng
topic `/oakd/rgb/preview/image_raw` với QoS Best Effort; cửa sổ YOLO riêng
hiển thị bounding box, class và confidence. Khi đủ ba goal, giữ cửa sổ để quay
đoạn kết; nhấn Ctrl+C trong terminal để đóng cả phiên. Script in thư mục
`~/bai05_runs/<timestamp>` chứa log và ảnh detection. Có thể đặt
`BAI05_OUTPUT_DIR`, `BAI05_YOLO_MODEL`, `BAI05_SERVER_DISPLAY` hoặc
`BAI05_SKIP_BUILD=1` trước lệnh; `BAI05_ROS_DOMAIN_ID` mặc định là 75.
Đối số như `run_route:=false` được chuyển thẳng cho `ros2 launch`. Script
khởi động map/AMCL, Nav2 rồi mới mở các cửa sổ; route bắt đầu sau khi giao
diện đã xuất hiện. Terminal và file `launch.log` trong thư mục run ghi cùng
một log.

Lệnh ROS 2 thuần tương đương (không tự sắp cửa sổ):

```bash
source /opt/ros/humble/setup.bash
source ~/turtlebot5_github_ws/install/setup.bash
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
| 1 — tây bắc | -8.5 | 8.5 | -1.57079632679 |
| 2 — tây nam | -8.5 | -8.5 | 0.0 |
| 3 — đông nam | 9.0 | -9.0 | 1.57079632679 |

Các điểm nằm trong vùng tự do của cùng thành phần bản đồ. Khoảng hở gần vật cản
tĩnh lần lượt khoảng 1,45 / 1,45 / 0,95 m. Đường ngắn nhất sơ bộ trên lưới map
với khoảng hở 0,44 m dài khoảng 12,8 / 21,4 / 23,8 m từ điểm xuất phát đến
Goal 1, Goal 1 đến Goal 2, và Goal 2 đến Goal 3. Đây là ước lượng chọn goal,
không phải quãng đường Nav2 đo được trong lượt chạy.

Sửa `config/goals.yaml` trước lượt mới nếu thay goal. Launch mặc định nạp map
từ package `bai04_turtlebot4`; bản sao trong `maps/` giữ nguyên từ Bài 04.
`worlds/maze_yolo.sdf` giữ tường của maze và bổ sung hai bình chữa cháy;
`worlds/placements.yaml` ghi vị trí một bình gần Goal 1 và một bình giữa tuyến Goal 2→3.
Scene giữ một người ở vị trí trống `(-3, 5)`; không chứa bàn, ghế hoặc người ở vị trí lỗi. Hai bình chữa cháy nằm gần Goal 1 và trên tuyến Goal 2→3 tại `(-1.8, -5.5)`.
Bình chữa cháy là model SDF dựng bằng hình học đơn giản, có collision để LiDAR/costmap
quan sát. Map `map.yaml`/`map.pgm` tuần 4 giữ nguyên; các vật thêm vào chỉ thuộc world
mô phỏng. Bình thứ hai đặt ở `(-1.8, -5.5)`, trên quỹ đạo cũ từ Goal 2
đến Goal 3. Chương trình gửi trực tiếp Goal 3 sau Goal 2, không có goal quan sát trung gian;
LiDAR đưa bình vào costmap cục bộ để bộ lập kế hoạch/bộ điều khiển tránh khi tới gần. YOLO
chỉ nhận diện hình ảnh, không điều khiển Nav2; box xuất hiện khi bình vào ảnh và vượt
confidence 0,12.

## Chạy riêng từng phần

```bash
# Mô phỏng + AMCL/Nav2 + YOLO, chưa gửi route
ros2 launch bai05_turtlebot4 demo.launch.py run_route:=false output_dir:=$HOME/bai05_run

# Trong terminal khác, cùng các biến ROS_DOMAIN_ID/IGN_PARTITION ở trên
ros2 run bai05_turtlebot4 navigate_goals.py --ros-args -p use_sim_time:=true \
  -p goals_file:=$HOME/turtlebot5_github_ws/src/bai05_turtlebot4/config/goals.yaml \
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

Mã bài tập do sinh viên thực hiện, Apache-2.0. Model Fuel, world upstream và
YOLO-World weights giữ quyền tác giả/điều khoản nguồn; xem `NOTICE`.
