# SDL
AAU Robotics 8 semester project

## Dependencies
install libfranka from [here](https://frankaemika.github.io/docs/installation_linux.html)
```
sudo apt install ros-humble-behaviortree-cpp ros-humble-realsense2-* ros-humble-librealsense2*
cd ~
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone -b humble https://github.com/BehaviorTree/BehaviorTree.ROS2
git clone https://github.com/tenfoldpaper/panda_ros2
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
colcon build --packages-ignore camera
colcon build --packages-select camera --symlink-install
source install/setup.bash
```

### pgsql_services and supabase_docker
Check seperate readme files for instructions. 


# todo?
## Running (in seprate terminals)
```
ros2 launch omtp_factory_moveit_config demo.launch.py

ros2 launch manipulator move_robot_server.launch.py
```


