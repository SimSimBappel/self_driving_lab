use the following to manually run FindArucoTag action.
This must also be done when running in debug mode to start the detector callback.
```
ros2 action  send_goal /detect_marker_pose behavior_tree_ros2_actions/action/FindArucoTag "{id: 11}"
```

# TODO
- [x] Implement timeout function for action FindArucoTag.
- [x] Implement realsense ros2 wrapper for image pipeline.
- [x] Make sure there is only one marker of same id.
- [x] Get camera info from topic.
- [x] Make the detection callable though action(s).