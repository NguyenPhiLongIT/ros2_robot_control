# 🤖 My Robot Simulation (ROS 2 + Gazebo + RViz)

Dự án này mô phỏng một robot di động 2 bánh (differential drive) với 1 bánh caster hỗ trợ cân bằng.  
Robot được mô hình hóa bằng **URDF/Xacro**, mô phỏng trong **Gazebo**, hiển thị trong **RViz** và có thể điều khiển thông qua **GUI + bàn phím**.

## ⚙️ Cài đặt

### 1️⃣ Clone workspace
```bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone <repo_url>
cd ~/ros2_ws
colcon build
source install/setup.bash

## Run: 
ros2 launch my_robot_description display.launch.py
