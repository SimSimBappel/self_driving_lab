import os
import cv2
import time
import glob
import rclpy
import numpy as np
from cv2 import aruco
import tf_transformations
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from behavior_tree_ros2_actions.action import FindArucoTag
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from aruco_pose_estimation.utils import ARUCO_DICT
from aruco_pose_estimation.pose_estimation import pose_estimation
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseArray
from aruco_interfaces.msg import ArucoMarkers
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class CameraSubscriber(Node):
    def __init__(self, image_topic, info_topic, debug=False):
        super().__init__('camera_subscriber')
        self.image_sub = self.create_subscription(
                Image, image_topic, self.image_callback, 1#qos_profile_sensor_data
            )
        self.image_sub
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self.info_callback, qos_profile_sensor_data
        )
        self.info_sub 
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        if debug:
            self.streaming = True
            print("streaming")
        else:    
            self.streaming = False
        self.img_raw = None
        self.img_copy = None
        self.img_none = False
        # self.markers = ArucoMarkers()
        self.pose_array = PoseArray()
        self.info_msg = None
        self.debug = debug
        self.datastamp = None

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # script_dir = os.path.dirname(os.path.abspath(__file__))
        # package_dir = os.path.dirname(script_dir)
        # self.calibration_file_path = os.path.join(package_dir, 'resource', 'calibration_data.npz')
        # print(self.calibration_file_path)
        # self.camera_matrix, self.distortion_coeffs = self.calib_cam(self.calibration_file_path)


    def info_callback(self, info_msg):
        self.info_msg = info_msg
        # get the intrinsic matrix and distortion coefficients from the camera info
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)

        self.logger.info("Camera info received.")
        self.logger.info("Intrinsic matrix: {}".format(self.intrinsic_mat))
        self.logger.info("Distortion coefficients: {}".format(self.distortion))
        self.logger.info("Camera frame: {}x{}".format(self.info_msg.width, self.info_msg.height))

        # Assume that camera parameters will remain the same...
        self.destroy_subscription(self.info_sub)

    def image_callback(self, data: Image):
        if not self.streaming:
            return

        if self.info_msg is None:
            self.logger.warn("No camera info has been received!")
            return
        
        try:
            # Convert ROS Image message to OpenCV image
            self.img_raw = self.bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
            self.img_copy = self.img_raw.copy()
            # self.markers = ArucoMarkers()
            self.pose_array = PoseArray()
            # if self.camera_frame == "":
            #     self.markers.header.frame_id = self.info_msg.header.frame_id
                # self.pose_array.header.frame_id = self.info_msg.header.frame_id
            # else:
            #     self.markers.header.frame_id = self.camera_frame
            #     self.pose_array.header.frame_id = self.camera_frame
            # self.markers.header.frame_id = 'camera_color_optical_frame'
            # self.pose_array.header.frame_id = 'camera_color_optical_frame'
            
            self.datastamp = data.header.stamp
            # self.markers.header.stamp = data.header.stamp
            # self.pose_array.header.stamp = data.header.stamp
        except Exception as e:
            self.logger.warn(str(e))
            return

    def get_latest_image(self):
        return self.img_raw

    def start_streaming(self):
        self.streaming = True

    def stop_streaming(self):
        if not self.debug:
            self.streaming = False

    # def calib_cam(self, calibration_file_path): 
    #     """Calibrate camera from file or images"""
    #     if os.path.exists(calibration_file_path):
    #         # Load calibration data from file
    #         with np.load(calibration_file_path) as data:
    #             mtx, dist = [data[i] for i in ('mtx', 'dist')]
    #         self.logger.info("Camera calibration data loaded from file.")
    #         return mtx, dist

    #     # If calibration file doesn't exist, perform calibration
    #     criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    #     objp = np.zeros((9*6,3), np.float32)
    #     objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    #     objpoints = [] # 3d point in real world space
    #     imgpoints = [] # 2d points in image plane.

    #     package_dir = os.path.abspath(__file__)
    #     for i in range(2): 
    #         package_dir = os.path.dirname(os.path.dirname(package_dir))

    #     image_path = os.path.join(package_dir, 'src', 'self_driving_lab', 'camera', 'resource', 'calib_images', 'checkerboard')
    #     images = glob.glob(os.path.join(image_path, '*.jpg'))
    #     print(image_path)

    #     for fname in images:
    #         self.logger.info(f"img: {fname}")
    #         img = cv2.imread(fname)
    #         gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    #         ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    #         if ret == True:
    #             objpoints.append(objp)
    #             corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    #             imgpoints.append(corners2)

    #     ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    #     self.logger.debug("matrix: " + str(mtx))
    #     self.logger.debug("distortion: " + str(dist))
    #     self.logger.info("Camera calibrated")

    #     # Save calibration data to file
    #     np.savez(calibration_file_path, mtx=mtx, dist=dist)
    #     self.logger.info("Camera calibration data saved to file.")

    #     return mtx, dist  # Return camera matrix and distortion coefficients


class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.logger = self.get_logger()
        self.logger.info('Initializing aruco_marker_detector')
        self.initialize_parameters()
        self.camera_subscriber = CameraSubscriber(self.image_topic, self.info_topic, self.debug)
        
        self.action_server = ActionServer(
            self,
            FindArucoTag,
            'detect_marker_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
            )
        self.timeout = 10 # Secounds
        self.found_object = False
        self.result = None
        self.aruco_size = 0.0435 # Meters
        self.camera_subscriber.start_streaming()
        if self.debug:
            self.poses_pub = self.create_publisher(PoseArray, self.markers_visualization_topic, 10)
            self.markers_pub = self.create_publisher(ArucoMarkers, self.detected_markers_topic, 10)
            self.image_pub = self.create_publisher(Image, self.output_image_topic, 10)


        

        # Make sure we have a valid dictionary id:
        try:
            dictionary_id = cv2.aruco.__getattribute__(self.dictionary_id_name)
            # check if the dictionary_id is a valid dictionary inside ARUCO_DICT values
            if dictionary_id not in ARUCO_DICT.values():
                raise AttributeError
        except AttributeError:
            self.logger.error(
                "bad aruco_dictionary_id: {}".format(self.dictionary_id_name)
            )
            options = "\n".join([s for s in ARUCO_DICT])
            self.logger.error("valid options: {}".format(options))
    
        # Set up fields for camera parameters
        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        # code for updated version of cv2 (4.7.0)
        self.aruco_dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.aruco_parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dictionary, self.aruco_parameters)

        if self.debug:
            print("Debug mode enabled")
            self.timer = self.create_timer(1.0, self.detector_callback)




    def destroy(self):
        """Shutdown all components of node"""
        self.camera_subscriber.destroy_node()
        self.action_server.destroy()
        super().destroy_node()
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        if goal_request.id > -1 and goal_request.id < 251:
            self.logger.info('Received goal request ID:' + str(goal_request.id))
            return GoalResponse.ACCEPT
        else:
            self.logger.info('Goal request ID:' + str(goal_request.id) + "is not in range!")
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.logger.info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def reset(self):
        """Delete image after using"""
        self.camera_subscriber.img_raw = None
        

    def execute_callback(self, goal_handle):
        self.logger.info("Looking for ID:" + str(goal_handle.request.id))
        self.found_object = False
        self.result = None
        start_time = time.time()
        self.camera_subscriber.start_streaming()
        count = 0

        while not goal_handle.is_cancel_requested and not self.found_object and time.time() - start_time < self.timeout:
            if self.camera_subscriber.img_raw is None:
                self.camera_subscriber.start_streaming()
                rclpy.spin_once(self.camera_subscriber, timeout_sec=0.1)
                count = count + 1
                if count == 4:
                    self.logger.warn("Image not retrieved!")
            else:
                if count >= 4:
                    self.logger.info("Image found")
                count = 0
                markers = None
                pose_array = None
                markers = ArucoMarkers()
                pose_array = PoseArray()
                markers.header.frame_id = 'camera_color_optical_frame'
                pose_array.header.frame_id = 'camera_color_optical_frame'
                markers.header.stamp = self.camera_subscriber.datastamp

                # frame, pose_array, markers = pose_estimation(rgb_frame=self.camera_subscriber.get_latest_image(), depth_frame=None,
                #                                                 aruco_detector=self.aruco_detector,
                #                                                 marker_size=self.marker_size, matrix_coefficients=self.camera_subscriber.camera_matrix,
                #                                                 distortion_coefficients=self.camera_subscriber.distortion_coeffs, pose_array=self.camera_subscriber.pose_array, markers=self.camera_subscriber.markers)

                _, __, markers = pose_estimation(rgb_frame=self.camera_subscriber.get_latest_image(), depth_frame=None,
                                                                aruco_detector=self.aruco_detector,
                                                                marker_size=self.marker_size, matrix_coefficients=self.camera_subscriber.intrinsic_mat,
                                                                distortion_coefficients=self.camera_subscriber.distortion, pose_array=pose_array, markers=markers)

                if markers.marker_ids is not None:
                    if markers.marker_ids.count(goal_handle.request.id) == 1:
                        # self.logger.info(f"{goal_handle.request.id} is in the array only once.")
                        index = markers.marker_ids.index(goal_handle.request.id)
                        marker_pose_msg = PoseStamped()
                        marker_pose_msg.header.stamp = self.get_clock().now().to_msg()
                        marker_pose_msg.header.frame_id = self.camera_frame
                        # marker_pose_msg.child_frame_id = f"marker_{goal_handle.request.id}"
                        marker_pose_msg.pose.position.x = markers.poses[index].position.x
                        marker_pose_msg.pose.position.y = markers.poses[index].position.y
                        marker_pose_msg.pose.position.z = markers.poses[index].position.z
                        marker_pose_msg.pose.orientation.x = -1.0 * markers.poses[index].orientation.x  # Flip the orientation by multiplying by -1.0
                        marker_pose_msg.pose.orientation.y = markers.poses[index].orientation.y
                        marker_pose_msg.pose.orientation.z = markers.poses[index].orientation.z
                        marker_pose_msg.pose.orientation.w = markers.poses[index].orientation.w
                        self.found_object = True
                        self.logger.info(str(marker_pose_msg))
                        self.logger.info(str(index))
                        result = FindArucoTag.Result()
                        result.marker_pose_msg = marker_pose_msg
                        self.logger.info("Found ID: " + str(goal_handle.request.id))
                        goal_handle.succeed()
                        self.camera_subscriber.stop_streaming()
                        self.reset()
                        return result

                    elif markers.marker_ids.count(goal_handle.request.id) > 1:
                        self.logger.warn(f"{goal_handle.request.id} is in the array more than once.")
                        self.logger.info(str(markers.marker_ids))
                        goal_handle.abort()
                        self.camera_subscriber.stop_streaming()
                        self.reset()
                        return FindArucoTag.Result()

                    elif self.debug:
                        self.logger.info(f"{goal_handle.request.id} is not in the array.")
                        self.logger.info(str(markers.marker_ids))
            
                self.reset()
                time.sleep(0.05)

        self.logger.warn(f"ID: {goal_handle.request.id} is not in the array. TIMEOUT")
        self.camera_subscriber.stop_streaming()
        self.reset()
        goal_handle.abort()
        return FindArucoTag.Result()
    

    def detector_callback(self):
        """Callback function for the timer"""
        rclpy.spin_once(self.camera_subscriber, timeout_sec=0.1)
        if self.camera_subscriber.img_copy is not None:
            markers = ArucoMarkers()
            pose_array = PoseArray()
            markers.header.frame_id = 'camera_color_optical_frame'
            pose_array.header.frame_id = 'camera_color_optical_frame'
            markers.header.stamp = self.camera_subscriber.datastamp
            pose_array.header.stamp = self.camera_subscriber.datastamp

            frame, pose_array, markers = pose_estimation(rgb_frame=self.camera_subscriber.img_raw, depth_frame=None,
                    aruco_detector=self.aruco_detector,
                    marker_size=self.marker_size, matrix_coefficients=self.camera_subscriber.intrinsic_mat,
                    distortion_coefficients=self.camera_subscriber.distortion, pose_array=pose_array, markers=markers)

            if len(markers.marker_ids) > 0:
                # Publish the results with the poses and markes positions
                # for i, marker in enumerate(pose_array.poses):
                    # pose_array.poses[i].orientation.z = -pose_array.poses[i].orientation.z  # Flip the orientation by multiplying by -1.0
                    # pose_array.poses[i].orientation.x = np.sin(np.pi/2) * pose_array.poses[i].orientation.x
                    # pose_array.poses[i].orientation.y = np.cos(np.pi/4) * pose_array.poses[i].orientation.y
                    # pose_array.poses[i].orientation.z = np.cos(np.pi/2) * pose_array.poses[i].orientation.z
                    # pose_array.poses[i].orientation.w = np.sin(np.pi/2) * -pose_array.poses[i].orientation.w
                self.poses_pub.publish(pose_array)
                self.markers_pub.publish(markers)
            else:
                self.logger.info("No markers detected")

            # publish the image frame with computed markers positions over the image
            self.image_pub.publish(self.camera_subscriber.bridge.cv2_to_imgmsg(frame, "rgb8"))
            self.camera_subscriber.img_copy = None
        else:
            print("No image received")
            

    def initialize_parameters(self):
        # Declare and read parameters from aruco_params.yaml
        self.declare_parameter(
            name="marker_size",
            value=0.0435,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the markers in meters.",
            ),
        )

        self.declare_parameter(
            name="debug",
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Publish debug topics",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_6X6_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Dictionary that was used to generate markers.",
            ),
        )

        self.declare_parameter(
            name="use_depth_input",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Use depth camera input for pose estimation instead of RGB image",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="depth_image_topic",
            value="/camera/depth/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Depth camera topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to.",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame to use.",
            ),
        )

        self.declare_parameter(
            name="detected_markers_topic",
            value="/aruco_markers",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish detected markers as array of marker ids and poses",
            ),
        )

        self.declare_parameter(
            name="markers_visualization_topic",
            value="/aruco_poses",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish markers as pose array",
            ),
        )

        self.declare_parameter(
            name="output_image_topic",
            value="/aruco_image",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish annotated images with markers drawn on them",
            ),
        )

        # read parameters from aruco_params.yaml and store them
        self.marker_size = (
            self.get_parameter("marker_size").get_parameter_value().double_value
        )
        self.logger.info(f"Marker size: {self.marker_size}")

        self.debug = (
            self.get_parameter("debug").get_parameter_value().bool_value
        )
        self.logger.info(f"Debug: {self.debug}")

        self.dictionary_id_name = (
            self.get_parameter("aruco_dictionary_id").get_parameter_value().string_value
        )
        self.logger.info(f"Marker type: {self.dictionary_id_name}")

        self.use_depth_input = (
            self.get_parameter("use_depth_input").get_parameter_value().bool_value
        )
        self.logger.info(f"Use depth input: {self.use_depth_input}")

        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )
        self.logger.info(f"Input image topic: {self.image_topic}")

        self.depth_image_topic = (
            self.get_parameter("depth_image_topic").get_parameter_value().string_value
        )
        self.logger.info(f"Input depth image topic: {self.depth_image_topic}")

        self.info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.logger.info(f"Image camera info topic: {self.info_topic}")

        self.camera_frame = (
            self.get_parameter("camera_frame").get_parameter_value().string_value
        )
        self.logger.info(f"Camera frame: {self.camera_frame}")

        # Output topics
        self.detected_markers_topic = (
            self.get_parameter("detected_markers_topic").get_parameter_value().string_value
        )

        self.markers_visualization_topic = (
            self.get_parameter("markers_visualization_topic").get_parameter_value().string_value
        )

        self.output_image_topic = (
            self.get_parameter("output_image_topic").get_parameter_value().string_value
        )




def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detector = ArucoMarkerDetector()
    try:
        rclpy.spin(aruco_marker_detector)
    except Exception as e: 
        print(e)
    aruco_marker_detector.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
