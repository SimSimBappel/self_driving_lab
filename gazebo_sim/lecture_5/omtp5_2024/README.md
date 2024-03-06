# Computer vision for grasping exercises:

In this set of exercises we will utilize a <b>ROS</b> [1] supported simulation environment (i.e. <b>Gazebo</b> [2]) environment that consists of a robotic arm, a RGB-D camera, a table and a set of tabletop objects.

The goal of the exercises is to build a image processing pipeline, that outputs the tabletop objects 6D pose, and use MoveIt! ROS interfaces to grasp the objects. 

[1] https://ros.org/ <br>
[2] https://gazebosim.org/home <br>

## 1. Image processing with ROS

In this lecture we will implement a complete image processing pipeline, that reads raw image topics from Gazebo or a ROS supported RGB-D camera, the intrisics camera matrices, and extrinsics, via known kinematics (i.e. transformations between color and depth image frames), and outputs rectified color and depth images, and a colored 3D point cloud.

### 1.1. <b>Setup a ROS local environment</b>
 
#### <b>Tasks</b>: <br>
1.1.1. Create a ROS workspace, <b>catkin_vision_\<group_name\>\_ws</b> , comprising a package entitled <b>computer_vision_\<group_name\>_pkg</b> and a folder for thirdparty packages. Consider the following structure:

            
            catkin_vision_<group_name>_ws/src/
            ├── computer_vision_<group_name>_pkg
            ├─────launch/
            │         ├── 1_visualizer.launch 
            │         ├── 2_image_processing.launch 
            │         ├── 3_object_detection.launch 
            │         ├── 4_object_pose_estimation.launch
            │         └── 5_vision_based_grasping.launch
            ├─────src/ 
            ├───────── .py, .c, .cpp files                     
            ├──── CMakeLists.txt   
            ├──── package.xml      
            ├──── README.md       
            └────requirements.txt
               

1.1.2. Create a <b>1_visualizer.launch</b> file that should spawn <b>rviz</b> [1] and/or <b>image viewer</b> [2] node(s) for visualizing the Gazebo camera depth and color images output. Add the correct dependencies to the CMakeLists.txt and package.xml files

Consider the <b>omtp_lecture5_base.launch</b> for a grasping simulation environment, and the <b>1_image_processing.launch</b>, as a starting basis for the exercise.

<b>Important</b>: consider the files under <b>omtp_lecture5</b>, as a starting basis for the exercise. Feel free to change the tabletop objects with different ones. E.g. the urdf/Ycb.

#### <b>References</b>: <br>
[1] http://wiki.ros.org/image_view <br>
[2] http://wiki.ros.org/rviz

### 1.2. <b>Image rectification and registration</b>

#### <b>Tasks</b>: <br>
1.2.1. Utilize <b>image_proc</b> [1] and <b>depth_image_proc</b> [2] ROS stacks, respectively to rectify the raw images, register the depth image in a common frame (e.g. the color image frame), and compute a colored point cloud.

#### <b>Relevant files</b>: <br>

  - ompt_lecture5/launch/2_image_processing.launch

Create a <b>2_visualizer.launch</b>  file with an <b>image_view</b> or <b>rviz</b> to allow visualizing the resulting rectification and registration output images.

#### <b>References</b>: <br>
[1] http://wiki.ros.org/image_proc <br>
[2] http://wiki.ros.org/depth_image_proc

## 2. Visual recognition with ROS

In this lecture the apprendice(s) will implement a 6D pose estimation pipeline, that reads rectified image topics from previous lecture, and outputs object 2D detections and 6D poses.

### 2.1. <b>2D object detection</b>

#### <b>Tasks</b>: <br>
Integrate an object detection algorithm within the image perception pipeline, that should receive 2D color images  and output bounding boxes (i.e. bounding boxes + object class)<br>
    - Basic: Utilize a pre-existing 2D object detection algorithm, implemented in ROS (e.g. [1],[2]).<br>
    - Advanced: Create your own ROS node that detects cubes (2D squares seen from a top view), and outputs object detections. Utilize the OpenCV library (bounding boxes + object class) using 2D images    

#### <b>References</b>: <br>
[1] http://wiki.ros.org/find_object_2d<br>
[2] https://github.com/raghavauppuluri13/yolov5_pytorch_ros


### 2.2. <b>6D object pose estimation</b>

#### <b>Tasks</b>: <br>
Integrate a 6D pose estimation algorithm from publicly available ROS supported packages
- Basic: Utilize a pre-existing 6D pose estimation algorithm, implemented in ROS (e.g. [1]),
- Advanced: Utilize the object detections from the previous exercise, depth images, and known camera intrinsics and extrinsics to estimate the 6D pose of tabletop objects.

### 2.3. <b>Vision-based grasping</b>

<b>Tasks</b>: <br>

- Basic: Use PCA analysis for grasp generation. Consider <b>numpy</b> [2] and the <b>PCL library</b> [3,4] for principal component analysis of tabletop objects.
- Advanced: utilize moveit simple grasps generator [5] or deep-learning based approaches [6], for grasp candidate generation.
- Integrate the developed modules with the moveit planning pipeline. Please refer to previous lectures.
- Optional: incorporate the whole perception pipeline with the moveit octomap─ for vision-based obstacle avoidance [7]
#### <b>References</b>: <br>
[1] https://github.com/paul-shuvo/ROS-Pose <br>
[2] https://numpy.org/ <br>
[3] https://pointclouds.org/ <br>
[4] https://python-pcl-fork.readthedocs.io/en/rc_patches4/install.html <br>
[5] https://ros-planning.github.io/moveit_tutorials/doc/moveit_grasps/moveit_grasps_tutorial.html <br>
[6] https://ros-planning.github.io/moveit_tutorials/doc/moveit_deep_grasps <br>
[7] https://ros-planning.github.io/moveit_tutorials/doc/perception_pipeline/perception_pipeline_tutorial.html
## Lecture deliverables:

 - Document and present the proposed solutions
 - Developed code

## Important reading(s)
Multiple View Geometry in Computer Vision Second Edition. Richard Hartley and Andrew Zisserman, Cambridge University Press, March 2004.
Zhang, Zhengyou. "A flexible new technique for camera calibration." IEEE Transactions on pattern analysis and machine intelligence 22.11 (2000)
## Lecturer: 
Rui Pimentel de Figueiredo
