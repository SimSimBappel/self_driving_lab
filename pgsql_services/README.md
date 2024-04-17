# Postgresql Services

## System dependencies
```
sudo apt-get install -y libpqxx-dev libpqxx-6.4
```

## Testing services
Please see docstrings in source code or .srv and .msg files for detailed compositions of services. 

## Todo
[x] - Internal relateSlots(), to relate increasing slot_ids to a local set. i.e. T1: 1, 2, 3 ; T2: 1 (4), 2 (5), 3 (6)
[ ] - Add static transforms to respective .csv files in ../data directory (aruco->slot1) and (slot1 -> rest)
[x] - Place vessel/chemical (upsert fnc) -> findslots() internal function
[ ] - Edge cases, what about when we grip the item? (remove)
[ ] - What is the best returned chem / vessel ? Maybe return a list sorted. 
[ ] - Various adding to system flows, like: workstation -> arucotag -> tray/machine -> (tray_slots) -> vessel/chemical -> placement_tables
[ ] - lookupAruco -> if needed
[ ] - Tidy up recurring code
[ ] - fix csv.h strcpy fnc -> null cases, if we feel like it. 


### Get Vessel
```
request: 
string name
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

#### Response Messages
Vessel found
Vessel has no placements
Vessel not found

#### Example CLI
```

ros2 service call /get_pre_pour_pose_service behavior_tree_ros2_actions/srv/GetPrePourPose "{object_id: 20}"
ros2 service call /get_vessel pgsql_interfaces/srv/GetVessel "{name: 'beaker'}"
---
pgsql_interfaces.srv.GetVessel_Response(name='robot_table', lookout_pose=geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711448538, nanosec=506852090), frame_id='panda_link0'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=2.0, z=3.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), aruco_id=1, aruco_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711448538, nanosec=505478335), frame_id='panda_link0'), child_frame_id='slot_1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.7000000000000001, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), slot_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711448538, nanosec=505489042), frame_id='panda_link0'), child_frame_id='', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), success=True, message='Vessel found')
```

### Get Chemical
```
request: 
string name
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

#### Response Messages
Chemical found
Chemical has no placements
Chemical not found

#### Example CLI
```
ros2 service call /get_chemical pgsql_interfaces/srv/GetChemical "{name: 'natrium_chloride'}"
---
pgsql_interfaces.srv.GetChemical_Response(workstation_name='robot_table', lookout_pose=geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711451806, nanosec=645658287), frame_id='panda_link0'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=2.0, z=3.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), aruco_id=2, aruco_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711451806, nanosec=644511923), frame_id='panda_link0'), child_frame_id='slot_1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.5, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), slot_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711451806, nanosec=644531086), frame_id='panda_link0'), child_frame_id='', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), empty=False, success=True, message='Chemical found')

```

### Place Vessel
```
request:
string name
int8 tray_id
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

#### Response Messages
Operation completed successfully
No free slots
Tray type is not vessel
Could not access database


#### Example CLI

```
ros2 service call /place_vessel pgsql_interfaces/srv/PlaceVessel "{name: 'beaker', tray_id: 1}"
---
pgsql_interfaces.srv.PlaceVessel_Response(workstation_name='robot_table', lookout_pose=geometry_msgs.msg.PoseStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711456628, nanosec=214935337), frame_id='panda_link0'), pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=1.0, y=2.0, z=3.0), orientation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), aruco_id=1, aruco_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711456628, nanosec=215027807), frame_id='panda_link0'), child_frame_id='slot_1', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=0.7000000000000001, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), slot_to_slot_transform=geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1711456628, nanosec=215033176), frame_id='panda_link0'), child_frame_id='slot_3', transform=geometry_msgs.msg.Transform(translation=geometry_msgs.msg.Vector3(x=3.0, y=0.0, z=0.0), rotation=geometry_msgs.msg.Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))), success=True, message='Operation completed successfully')
```


### Remove Chemical Placement

```
request:
string name
int8 slot_id
---
response:
bool success
string message
```

#### Response Messages
Operation completed successfully
No chemical found in the specified slot"
Chemical not found 
Could not access database


#### Example CLI

```
ros2 service call /remove_chemical_placement pgsql_interfaces/srv/RemoveChemicalPlacement "{name: 'natrium_chloride, slot_id: 4}"
---
pgsql_interfaces.srv.RemoveChemicalPlacement_Response(success=True, message='Operation completed successfully')
```

### Remove Vessel Placement

```
request:
string name
int8 slot_id
---
response:
bool success
string message
```

#### Response Messages
Operation completed successfully
No vessel found in the specified slot"
Vessel not found 
Could not access database


#### Example CLI

```
ros2 service call /remove_vessel_placement pgsql_interfaces/srv/RemoveVesselPlacement "{name: 'beaker, slot_id: 4}"
---
pgsql_interfaces.srv.RemoveVesselPlacement_Response(success=True, message='Operation completed successfully')
```

