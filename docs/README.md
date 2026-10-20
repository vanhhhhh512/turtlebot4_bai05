# Báo cáo Bài thực hành 05 — Đỗ Việt Anh, 23020719

- [Video demo](https://drive.google.com/drive/folders/1GKdy0U0tKGbxUAaGAmAffTPxJa5pT8wy?usp=drive_link)
- [Báo cáo PDF — 16 trang](report/Bao_cao_23020719_Bai_thuc_hanh_05.pdf)
- [ZIP LaTeX mở/sửa trên Prism](report/Bao_cao_23020719_Bai05_LaTeX_Prism.zip)
- [GitHub mã nguồn Bài 05](https://github.com/vanhhhhh512/turtlebot4_bai05)
- [GitHub Bài 04 — map/localization](https://github.com/vanhhhhh512/turtlebot4_bai04)

Báo cáo cập nhật ngày 20/10/2026 giữ 11 mục chính. Số liệu và ảnh camera lấy từ
`direct_goal_route_run10`, chạy source điều hướng commit `c87263f`. Cả ba goal
đều SUCCEEDED, MISSION_COMPLETE ghi 0 contact và 0 bump. Goal 3 được gửi trực tiếp
sau Goal 2, có thời gian dừng giữa goal nhưng không có pose quan sát trung gian.

Scene hiện tại có một người ở (-3, 5), bình gần Goal 1 (-7.8, 6.2) và bình trên
quỹ đạo Goal 2 → 3 (-1.8, -5.5). YOLO-World dùng confidence 0.12, imgsz 960;
log ghi 132 box person và 160 box fire extinguisher qua 1.176 frame đã xử lý.
Khoảng cách nhỏ nhất giữa tâm robot và tâm bình Goal 3 là 0.618 m (không phải
khoảng hở bề mặt). Chênh lệch AMCL/Gazebo lớn nhất ở các pose đã lưu là 0.194 m;
hai timestamp có thể khác nhau, nên không coi đó là accuracy đồng bộ độc lập.

## Nội dung

- `report/`: PDF, LaTeX mẫu và LaTeX đã điền số liệu, logo/font hỗ trợ.
- `results/`: biểu đồ, ảnh camera dùng trong PDF, metrics, validation và provenance.
- `data/direct_goal_route_run10/`: toàn bộ log, goal results, telemetry,
  detections, world runtime và ảnh raw/YOLO đã lưu của lượt kiểm chứng cuối.
- `tools/`: script tính số liệu/vẽ biểu đồ và dựng PDF.
- `publication.json`: link demo, GitHub và commit code đã kiểm chứng.

Ảnh mục 7.2 từ corners_run04 có camera OAK-D RGB hiển thị ngay trong RViz; đây là ảnh minh họa giao diện được ghi rõ nguồn. Bảng goal,
quỹ đạo, ảnh YOLO và số liệu kết quả thuộc run10; không ghép ảnh cũ thành minh
chứng cho scene mới. Link Video demo do sinh viên cung cấp.

## Dựng lại báo cáo

Cần Python 3 với NumPy, Matplotlib, Pillow và Tectonic (trong PATH hoặc
`~/.local/bin/tectonic`). Từ thư mục `docs` của repository:

```bash
python3 tools/analyze_run.py data/direct_goal_route_run10
python3 tools/make_report.py
python3 tools/package_latex.py
```

Script dùng nguyên map ở `../maps/`, đọc log đã có và tạo lại `results/` cùng
`report/Bao_cao_23020719_Bai_thuc_hanh_05.pdf`. Không khởi chạy ROS/Gazebo.
Cách cài đặt, build và chạy robot nằm trong [README chính](../README.md).
