# SDL
AAU Robotics 8 semester project

## Dependencies
install libfranka from [here](https://frankaemika.github.io/docs/installation_linux.html)
get the 0.9.2 version during the git checkout command
```
pip3 install opencv-python opencv-contrib-python transforms3d
sudo apt install ros-humble-behaviortree-cpp ros-humble-realsense2-* ros-humble-librealsense2* ros-humble-tf-transformations
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone -b humble https://github.com/BehaviorTree/BehaviorTree.ROS2
git clone https://github.com/tenfoldpaper/panda_ros2
git clone https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation # may need to be with --symlink-install
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
cd self_driving_lab/gazebo_sim
git clone -b foxy-devel https://github.com/pal-robotics/realsense_gazebo_plugin.git
cd ..
cd ..
cd ..
sudo apt install ros-humble-gazebo-ros2-control
colcon build --packages-ignore camera
colcon build --packages-select camera --symlink-install
source install/setup.bash
```

### pgsql_services and supabase_docker
Check seperate readme files for instructions. 


# use?
## Running
```
ros2 launch manipulator system.launch.py
```


