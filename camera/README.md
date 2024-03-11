use the following to manually run FindArucoTag action
```
ros2 action  send_goal /detect_marker_pose behavior_tree_ros2_actions/action/FindArucoTag "{id: 11}"
```

# TODO
- [x] Camera calibration uses relative folder.
- [x] Implement timeout function for action FindArucoTag.
- [x] Implement realsense ros2 wrapper for image pipeline.
- [ ] make sure there is only one marker of same id.
- [x] Get camera info from file.
- [x] Make the detection callable though action(s).
- [x] 