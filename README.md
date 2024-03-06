# SDL
AAU Robotics 8 semester project

## Dependencies
install libfranka from [here](https://frankaemika.github.io/docs/installation_linux.html)
```
sudo apt install ros-humble-behaviortree-cpp
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
cd ..
colcon build
source install/setup.bash
```

