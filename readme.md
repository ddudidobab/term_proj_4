# 자동항법 TERM PROJ - team 4

### build
```bash
mkdir proj_ws
cd proj_ws
mkdir src
cd src
git clone https://github.com/ddudidobab/term_proj_4.git
cd ..
colcon build --packages-select term_proj_4
source install/setup.bash
```

### run
```bash
#terminal 1
cd ~/proj_ws/src/term_proj_4
source install/setup.bash
rviz2 -d rviz_cfg.rviz
```
```bash
#terminal 2
cd ~/proj_ws
source install/setup.bash
ros2 launch term_proj_4 main.launch.py
```
