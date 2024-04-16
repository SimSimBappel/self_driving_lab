# Xdl_parser


To run this simply run the following command:

```sh
ros2 run xdl_parser2 xdl_parser_server
```

The request input is only a string, you can send an empty string for a file explorer to pop up to manually select the xdl. The parser returns True to the service call if the xdl is accepted and parsed.

If the xdl contains any procedure steps not defined it will return a false as the result and not send anything to the behaviour server

CLI service call:

```sh
ros2 service call /convert_xdl behavior_tree_ros2_actions/srv/Xdl "{xdl:""}"
```