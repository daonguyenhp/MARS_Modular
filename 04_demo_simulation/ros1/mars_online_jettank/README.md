# MARS Online JetTank Simulation (ROS 1)

Gói mô phỏng trực tuyến thuật toán **MARS** (Sequences of Bundles of Line Segments) cho robot **Hiwonder JetTank** trên nền tảng **ROS 1** (Ubuntu 18.04 / 20.04 - ROS Melodic / Noetic).

Robot tự động cảm nhận môi trường bằng LiDAR 360°, trích xuất các chùm tia (bundle), phát hiện ngõ cụt (BAR - Blind Alley Region) và tự động thoát ra theo phễu DAP (Diverging-Angle Point) một cách tối ưu.

---

## 1. Cấu trúc gói

```text
mars_online_jettank/
├── CMakeLists.txt              # Catkin build script (liên kết thư viện C++17 mars::demo_simulation)
├── package.xml                 # ROS 1 package manifest
├── launch/
│   └── jettank_paper_alley.launch   # Khởi động Gazebo, spawn JetTank, chạy online node & RViz
├── worlds/
│   └── paper_alley.world       # Bản đồ ngõ cụt 10m x 10m chuẩn Paper (16 bức tường)
├── urdf/
│   └── jettank.urdf.xacro      # Mô hình robot JetTank tích hợp Diff Drive & Lidar sensor
├── rviz/
│   └── jettank_paper_alley.rviz # Cấu hình RViz hiển thị Robot, LiDAR, Map, Trail & Bundles
├── include/
│   └── laser_geometry.hpp      # Trích xuất polygon từ sensor_msgs/LaserScan
└── src/
    ├── laser_geometry.cpp
    └── online_jettank_node.cpp # Node điều khiển bám sát thuật toán MARS
```

---

## 2. Giao tiếp Topics & Khung toạ độ

- **Lệnh điều khiển:** Xuất đồng thời ra `/hiwonder_controller/cmd_vel` và `/cmd_vel` (phù hợp cả JetTank thực tế lẫn Gazebo diff drive).
- **Cảm biến LiDAR:** Lắng nghe `/scan_raw` (hoặc `/scan`).
- **Odometry:** Lắng nghe `/odom`.
- **Trực quan hoá RViz:**
  - `/mars_online/markers`: Hiển thị các chùm tia (bundles), mục tiêu mở và đường ngắm.
  - `/mars_online/trail`: Quỹ đạo thực tế đã di chuyển (màu cam).
  - `/mars_online/entry`: Quỹ đạo lúc đi vào ngõ cụt (màu đỏ cam).
  - `/mars_online/return`: Quỹ đạo lúc thoát ngõ cụt theo DAP (màu xanh dương).
  - `/mars_online/plan`: Bước dự tính tiếp theo (màu xanh lá).
  - `/mars_online/map`: Bản đồ ô lưới tích luỹ từ LiDAR.

---

## 3. Hướng dẫn Biên dịch (Build)

1. Sao chép hoặc tạo symlink thư mục `MARS_Modular` hoặc `mars_online_jettank` vào `catkin_ws/src`:
   ```bash
   cd ~/catkin_ws/src
   # Hoặc trong workspace hiwonder/ros_ws:
   ln -s /path/to/MARS_Modular/04_demo_simulation/ros1/mars_online_jettank .
   ```

2. Biên dịch bằng `catkin_make`:
   ```bash
   cd ~/catkin_ws
   catkin_make -DCMAKE_BUILD_TYPE=Release
   source devel/setup.bash
   ```

---

## 4. Hướng dẫn Chạy Mô phỏng (Run)

Khởi động toàn bộ mô phỏng Gazebo + JetTank + MARS Navigator + RViz:
```bash
roslaunch mars_online_jettank jettank_paper_alley.launch
```

### Tuỳ chọn nâng cao:
- **Dùng mô hình từ gói `hiwonder_description` có sẵn của xe:**
  ```bash
  roslaunch mars_online_jettank jettank_paper_alley.launch use_hiwonder_pkg:=true
  ```
- **Chạy không bật giao diện Gazebo (để nhẹ máy, chỉ xem RViz):**
  ```bash
  roslaunch mars_online_jettank jettank_paper_alley.launch gui:=false
  ```
- **Thay đổi toạ độ Goal đích:**
  ```bash
  roslaunch mars_online_jettank jettank_paper_alley.launch goal_x:=8.6 goal_y:=6.8
  ```
- **Gửi toạ độ đích mới trực tiếp bằng công cụ "2D Nav Goal" trên thanh công cụ RViz.**
