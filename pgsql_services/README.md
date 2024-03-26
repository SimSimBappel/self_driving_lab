# Postgresql Services

## System dependencies
```
sudo apt-get install -y libpqxx-dev libpqxx-6.4
```

## Testing services
Please see docstrings in source code or .srv and .msg files for detailed compositions of services. 

## Todo
[ ] - Internal relateSlots(), to relate increasing slot_ids to a local set. i.e. T1: 1, 2, 3 ; T2: 1 (4), 2 (5), 3 (6)
[ ] - Add static transforms to respective .csv files in ../data directory (aruco->slot1) and (slot1 -> rest)
[ ] - Place vessel/chemical (upsert fnc) -> findslots() internal function
[ ] - Various adding to system flows, like: workstation -> arucotag -> tray/machine -> (tray_slots) -> vessel/chemical -> placement_tables
[ ] - lookupAruco -> if needed
[ ] - Tidy up recurring code
[ ] - fix csv.h strcpy fnc -> null cases, if we feel like it. 



### Get Vessel
```
request: 
'name' 
---
response:
string workstation_name
geometry_msgs/PoseStamped lookout_pose               
int8 aruco_id                                                       
geometry_msgs/TransformStamped aruco_to_slot_transform              
geometry_msgs/TransformStamped slot_to_slot_transform         
bool success
string message
```

#### Example CLI
```
ros2 service call /get_vessel pgsql_interfaces/srv/GetVessel "{name: 'beaker'}"
---
pgsql_interfaces.srv.GetVessel_Response(name='robot_table', lookout_pose=geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711448538, nanosec=506852090), frame_id='panda_link0'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=2.0, z=3.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), aruco_id=1, aruco_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711448538, nanosec=505478335), frame_id='panda_link0'), child_frame_id='slot_1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.7000000000000001, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), slot_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711448538, nanosec=505489042), frame_id='panda_link0'), child_frame_id='', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), success=True, message='Vessel found')
```

### Get Chemical
```
request: 
'name' 
---
response:
string workstation_name
geometry_msgs/PoseStamped lookout_pose               
int8 aruco_id                                                       
geometry_msgs/TransformStamped aruco_to_slot_transform              
geometry_msgs/TransformStamped slot_to_slot_transform   
bool empty      
bool success
string message
```

#### Example CLI
```
ros2 service call /get_chemical pgsql_interfaces/srv/GetChemical "{name: 'natrium_chloride'}"
---
pgsql_interfaces.srv.GetChemical_Response(workstation_name='robot_table', lookout_pose=geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711451806, nanosec=645658287), frame_id='panda_link0'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=2.0, z=3.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), aruco_id=2, aruco_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711451806, nanosec=644511923), frame_id='panda_link0'), child_frame_id='slot_1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.5, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), slot_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711451806, nanosec=644531086), frame_id='panda_link0'), child_frame_id='', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), empty=False, success=True, message='Chemical found')

```




