use the following to manually run FindArucoTag action
```
ros2 action  send_goal /detect_marker_pose behavior_tree_ros2_actions/action/FindArucoTag "{id: 11}"
```

# TODO
- [ ] Camera calibration uses relative folder.
- [ ] Implement timeout function for action FindArucoTag.
- [ ] Implement realsense ros2 wrapper for image pipeline.
- [ ] make sure there is only one marker of same id.
- [ ] Get camera info from file.