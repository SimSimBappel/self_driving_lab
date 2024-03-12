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
from geometry_msgs.msg import TransformStamped
from behavior_tree_ros2_actions.action import FindArucoTag
from rclpy.action import ActionServer, CancelResponse, GoalResponse


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.callback,
            10)
        self.subscription 
        self.bridge = CvBridge()
        self.logger = self.get_logger()
        self.streaming = False
        self.img_raw = None
        self.img_none = False
        script_dir = os.path.dirname(os.path.abspath(__file__))
        package_dir = os.path.dirname(script_dir)
        self.calibration_file_path = os.path.join(package_dir, 'resource', 'calibration_data.npz')
        self.camera_matrix, self.distortion_coeffs = self.calib_cam(self.calibration_file_path)

    def callback(self, data):
        if not self.streaming:
            return

        try:
            # Convert ROS Image message to OpenCV image
            self.img_raw = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.logger.warn(str(e))
            return

    def get_latest_image(self):
        return self.img_raw

    def start_streaming(self):
        self.streaming = True

    def stop_streaming(self):
        self.streaming = False

    def calib_cam(self, calibration_file_path): 
        """Calibrate camera from file or images"""
        if os.path.exists(calibration_file_path):
            # Load calibration data from file
            with np.load(calibration_file_path) as data:
                mtx, dist = [data[i] for i in ('mtx', 'dist')]
            self.logger.info("Camera calibration data loaded from file.")
            return mtx, dist

        # If calibration file doesn't exist, perform calibration
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        package_dir = os.path.abspath(__file__)
        for i in range(2): 
            package_dir = os.path.dirname(os.path.dirname(package_dir))

        image_path = os.path.join(package_dir, 'src', 'self_driving_lab', 'camera', 'resource', 'calib_images', 'checkerboard')
        images = glob.glob(os.path.join(image_path, '*.jpg'))

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, (7,6),None)
            if ret == True:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

        self.logger.debug("matrix: " + str(mtx))
        self.logger.debug("distortion: " + str(dist))
        self.logger.info("Camera calibrated")

        # Save calibration data to file
        np.savez(calibration_file_path, mtx=mtx, dist=dist)
        self.logger.info("Camera calibration data saved to file.")

        return mtx, dist  # Return camera matrix and distortion coefficients


class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.logger = self.get_logger()
        self.logger.info('Initializing aruco_marker_detector')
        self.camera_subscriber = CameraSubscriber()
        
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

    def destroy(self):
        """Shutdown all components of node"""
        self.camera_subscriber.destroy_node()
        self.action_server.destroy()
        super().destroy_node()
    
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        if goal_request.id > -1 and goal_request.id < 251:
            self.get_logger().info('Received goal request ID:' + str(goal_request.id))
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info('Goal request ID:' + str(goal_request.id) + "is not in range!")
            return GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def reset(self):
        """Delete image after using"""
        self.camera_subscriber.img_raw = None

    def execute_callback(self, goal_handle):
        """Get image and return transform to aruco tag"""
        self.logger.info("Looking for ID:" + str(goal_handle.request.id))
        self.found_object = False
        self.result = None
        start_time = time.time()
        self.camera_subscriber.start_streaming()
        count = 0

        while not goal_handle.is_cancel_requested and not self.found_object and time.time() - start_time < self.timeout: 
            # Process the images until the action is completed
            frame = self.camera_subscriber.get_latest_image()
            if self.camera_subscriber.img_raw is None:
                rclpy.spin_once(self.camera_subscriber, timeout_sec=0.1)
                count = count + 1
                if count == 4:
                    self.logger.warn("Image not retrieved!")
            else:
                if count >= 4:
                    self.logger.info("Image found")
                count = 0
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                parameters = aruco.DetectorParameters_create()
                corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

                if ids is not None and np.sum(ids == goal_handle.request.id) == 1:
                    index = np.where(ids == goal_handle.request.id)[0][0]
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], self.aruco_size, self.camera_subscriber.camera_matrix, self.camera_subscriber.distortion_coeffs)
                    if rvec is not None and tvec is not None:
                        marker_pose_msg = TransformStamped()
                        marker_pose_msg.header.stamp = self.get_clock().now().to_msg()
                        marker_pose_msg.header.frame_id = "camera"
                        marker_pose_msg.child_frame_id = f"marker_{goal_handle.request.id}"
                        marker_pose_msg.transform.translation.x = tvec[0][0][0]
                        marker_pose_msg.transform.translation.y = tvec[0][0][1]
                        marker_pose_msg.transform.translation.z = tvec[0][0][2]
                        q = tf_transformations.quaternion_from_euler(rvec[0][0][0], rvec[0][0][1], rvec[0][0][2])
                        marker_pose_msg.transform.rotation.x = q[0]
                        marker_pose_msg.transform.rotation.y = q[1]
                        marker_pose_msg.transform.rotation.z = q[2]
                        marker_pose_msg.transform.rotation.w = q[3]
                        self.found_object = True
                        result = FindArucoTag.Result()
                        result.marker_pose_msg = marker_pose_msg
                        self.logger.info("Found ID: " + str(goal_handle.request.id))
                        goal_handle.succeed()
                        self.reset()
                        self.camera_subscriber.stop_streaming()
                        return result
                elif ids is not None and goal_handle.request.id in ids and not np.sum(ids == goal_handle.request.id) == 1:
                    print(ids)
                    print(np.sum(ids == goal_handle.request.id))
                    self.logger.warn("More than one of id:" + str(goal_handle.request.id) + "found!")
                    goal_handle.abort()
                    self.reset()
                    self.camera_subscriber.stop_streaming()
                    return FindArucoTag.Result()

                self.reset()
                time.sleep(0.05)
        
        self.camera_subscriber.stop_streaming()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            return FindArucoTag.Result()
        elif not time.time() - start_time < self.timeout:
            goal_handle.abort()
            self.get_logger().warn('Timeout')
            return FindArucoTag.Result()
        else:
            cv2.destroyAllWindows()
            self.logger.error("wtf")
            self.destroy()


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
