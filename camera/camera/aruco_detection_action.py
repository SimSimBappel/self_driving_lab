import rclpy
from rclpy.action import ActionServer
import tf2_ros
import tf_transformations
from rclpy.node import Node
# from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from cv2 import aruco
import glob
from behavior_tree_ros2_actions.action import FindArucoTag  # Import your action type



class ArucoMarkerDetector(Node):
    def __init__(self):
        super().__init__('aruco_marker_detector')
        self.logger = self.get_logger()
        self.logger.info('Initializing aruco_marker_detector')
        self.publisher = self.create_publisher(TransformStamped, 'marker_pose', 10)
        # self.subscription = self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, 10)
        # self.action_server = self.create_action_server(Fibonacci, 'detect_marker_pose', self.execute_callback)
        self.action_server = ActionServer(
            self,
            FindArucoTag,
            'detect_marker_pose',
            execute_callback=self.execute_callback
        )
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        # self.marker_id_to_publish = 11  # Change this to the ID you want to publish
        self.camera_matrix, self.distortion_coeffs = self.calib_cam()  # Calibrate the camera

    def calib_cam(self):
        ####---------------------- CALIBRATION ---------------------------
        # termination criteria for the iterative algorithm
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        # checkerboard of size (7 x 6) is used
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # iterating through all calibration images
        # in the folder
        images = glob.glob('/home/simon/self_driving_ws/src/self_driving_lab/camera/resource/calib_images/checkerboard/*.jpg')
        
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # find the chess board (calibration pattern) corners
            ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

            # if calibration pattern is found, add object points,
            # image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                # Refine the corners of the detected corners
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)


        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        # if debug:
        #     print(mtx)
        self.logger.info("camera calibrated")
        return mtx, dist  # Return camera matrix and distortion coefficients
    
    def execute_callback(self, goal_handle):
        cap = cv2.VideoCapture(0)  # implement realsense_ros2
        self.logger.info("looking for " + str(goal_handle.request.id))
        while True: #implement timeout
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None and goal_handle.request.id in ids:
                index = np.where(ids == goal_handle.request.id)[0][0]
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], 0.0435, self.camera_matrix, self.distortion_coeffs)
                if rvec is not None and tvec is not None:
                    # Construct and publish the marker pose
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
                    self.publisher.publish(marker_pose_msg)
                    result = FindArucoTag.Result()
                    result.marker_pose_msg = marker_pose_msg
                    goal_handle.succeed()
                    cap.release()
                    cv2.destroyAllWindows()
                    return result



def main(args=None):
    rclpy.init(args=args)
    aruco_marker_detector = ArucoMarkerDetector()
    rclpy.spin(aruco_marker_detector)
    aruco_marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
