# SDL
AAU Robotics 8 semester project

## 3D models
[Onshape Link](https://cad.onshape.com/documents/302d39dc0d22175e2aaec147/w/40df758372de53e0b679b2a3/e/3cc2fa0ae1f39eaf5d8ddebc?renderMode=0&uiState=660e6a81d440c26a82b7aa22)

## Dependencies
install libfranka from [here](https://frankaemika.github.io/docs/installation_linux.html)
get the 0.9.2 version during the git checkout command
```
pip3 install opencv-python opencv-contrib-python transforms3d
sudo apt install ros-humble-behaviortree-cpp ros-humble-realsense2-* ros-humble-librealsense2* ros-humble-tf-transformations ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-joint-state-publisher ros-humble-gazebo-ros ros-humble-moveit ros-humble-graph-msgs ros-humble-moveit-visual-tools ros-humble-ament-cmake-clang-format ros-humble-xacro
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone -b humble https://github.com/BehaviorTree/BehaviorTree.ROS2
git clone https://github.com/tenfoldpaper/panda_ros2
git clone -b humble https://github.com/ros-planning/moveit_task_constructor.git
git clone https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation
cd ..
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/home/$USER/libfranka/build
source install/setup.bash
```

## Install
```
cd ~
mkdir -p sdl_ws/src
cd sdl_ws/src
git clone https://github.com/SimSimBappel/self_driving_lab
colcon build
source install/setup.bash
colcon build
source install/setup.bash
```

### pgsql_services and supabase_docker
Check seperate readme files for instructions. 


# use?
## Running
```
ros2 launch manipulator system.launch.py
```


