$ check_urdf omtp.urdf
 remeber in rviz to set that robotmodel should come from the robot:description

 $ xacro omtp_custom_factory.urdf.xacro > omtp_custom.urdf.xacro   (important command)

 If the robot do not show up in the rviz and gazebo, then remkae the urdf from the xacro file

 we might be able to use moveit_py from iron distrubution by building it from source