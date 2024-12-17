# for ROS
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int32  

# for CV
import cv2
import numpy as np

class nAruco(Node):
    def __init__(self):
        super().__init__('aruco')
        self.angle_publisher_ = self.create_publisher(Float32MultiArray, 'angle', 10)
        self.id_publisher_ = self.create_publisher(Int32, 'id', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.vector = [0.0, 0.0, 0.0]
        self.id = -1

        self.aruco_board_type = cv2.aruco.DICT_4X4_250
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(self.aruco_board_type)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        self.camera = {}
        self.open_camera()
        self.camera_matrix = np.array([[883.54778219,   0,         317.02516042], [  0,        883.13166889, 224.28063539], [  0,           0,         1.        ]], dtype=np.float32)
        self.camera_dist_coeffs = np.array([[-8.82651699e-01,  2.51857887e+00, -9.06745177e-05,  1.63195259e-03,  -6.79037934e+00]], dtype=np.float32)
        cv2.namedWindow("aruco")


    def timer_callback(self):
        marker = self.read_marker()
        if marker is None:
            self.get_logger().error('failed to read marker, skip.')
            return

        self.vector = marker[0]
        self.id = marker[1]

        msg = Float32MultiArray()
        msg.data = self.vector
        self.angle_publisher_.publish(msg)
        self.get_logger().info('Publishing angle: "%s"' % msg.data)

        msg = Int32()
        msg.data = self.id
        self.id_publisher_.publish(msg)
        self.get_logger().info('Publishing id: %d' % msg.data)

    def open_camera(self, cam_num=0):
        self.camera = cv2.VideoCapture(cam_num) 
        if not self.camera.isOpened():
            self.get_logger().error('failed to open camera')
        return self.camera.isOpened()
    
    def read_marker(self):
        if not self.camera.isOpened():
            if not self.open_camera():
                self.get_logger().error('no camera available')
                return None
        
        ret, img = self.camera.read()
        if not ret:
            self.get_logger().error('failed to get image frame')
            return None
        cv2.imshow("aruco", img)
        cv2.waitKey(1)
        
        corners, ids, _ = self.aruco_detector.detectMarkers(img)
        if ids is None or len(corners) == 0:
            self.get_logger().error('no marker detected')
            return None
        i = 0
        id = ids[i][0]

        #rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.camera_dist_coeffs)
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.camera_dist_coeffs)
        corner = np.array(corners[i]).reshape((4, 2))
        rmat, _ = cv2.Rodrigues(rvecs[i])
        normal_vector = -rmat[:, 2]
        euler_angle = cv2.RQDecomp3x3(rmat)[0]

        self.draw_marker_window(img, corner, id, normal_vector, rvecs[i], tvecs[i])
        #self.draw_marker_window_new(img, corner)
        if not is_marker_center(img, corner):
            self.get_logger().error('marker is not on center')
            return None
        
        angle1 = 0.0
        if euler_angle[0] < 0:
            angle1 = euler_angle[0] + 180
        else:
            angle1 = 180 - euler_angle[0]
        return ([float(angle1), float(euler_angle[1]), float(euler_angle[2])], int(id))
        #return ([float(normal_vector[0]), float(normal_vector[1]), float(normal_vector[2])], int(id))

    def draw_marker_window_new(self, img, corner):
        rvecs, tvecs, _ = my_estimatePoseSingleMarkers(corner, 0.05, self.camera_matrix, self.camera_dist_coeffs)
        cv2.aruco.drawDetectedMarkers(img, [corner])
        cv2.aruco.drawAxis(img, self.camera_matrix, self.camera_dist_coeff, rvecs[0], tvecs[0], 0.01)  
        cv2.imshow("aruco", img)
        cv2.waitKey(1)


    def draw_marker_window(self, img, corner, id, normal_vector, rvec, tvec):
        blue_BGR = (255, 0, 0)
        green_BGR = (0, 255, 0)  # 법선 벡터
        (topLeft, topRight, bottomRight, bottomLeft) = corner

        topRightpoint = (int(topRight[0]), int(topRight[1]))
        topLeftpoint = (int(topLeft[0]), int(topLeft[1]))
        bottomRightpoint = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeftpoint = (int(bottomLeft[0]), int(bottomLeft[1]))

        cv2.circle(img, topLeftpoint, 4, blue_BGR, -1)
        cv2.circle(img, topRightpoint, 4, blue_BGR, -1)
        cv2.circle(img, bottomRightpoint, 4, blue_BGR, -1)
        cv2.circle(img, bottomLeftpoint, 4, blue_BGR, -1)

        cv2.line(img, topLeftpoint, topRightpoint, blue_BGR, 2)
        cv2.line(img, topRightpoint, bottomRightpoint, blue_BGR, 2)
        cv2.line(img, bottomRightpoint, bottomLeftpoint, blue_BGR, 2)
        cv2.line(img, bottomLeftpoint, topLeftpoint, blue_BGR, 2)

        cv2.putText(img, f"ID: {id}",(topLeftpoint[0],topLeftpoint[1] - 10), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, blue_BGR, 2)

        end_point, _ = cv2.projectPoints(np.array([normal_vector * 0.05]), rvec, tvec, self.camera_matrix, self.camera_dist_coeffs)
        endpoint = (int(end_point[0][0][0]), int(end_point[0][0][1])) 
        centerX = int((topLeft[0]+bottomRight[0]) / 2)
        centerY = int((topLeft[1]+bottomRight[1]) / 2)
        centerpoint = (centerX, centerY)
        cv2.arrowedLine(img, centerpoint, endpoint, green_BGR, 2, tipLength=0.2)

        normal_text = f"({normal_vector[0]:.2f}, {normal_vector[1]:.2f}, {normal_vector[2]:.2f})"
        cv2.putText(img, normal_text, (centerX + 10, centerY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, green_BGR, 2)
        cv2.putText(img, "center : {}" % is_marker_center(img, corner), (10, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, green_BGR, 2)

        cv2.imshow("aruco", img)
        cv2.waitKey(1)

def is_marker_center(img, corner, margin_ratio = 0.05):
    (topLeft, topRight, bottomRight, bottomLeft) = corner
    corner_center_x = int((topLeft[0]+bottomRight[0]) / 2)
    corner_center_y = int((topLeft[1]+bottomRight[1]) / 2)
    image_height, image_width = img.shape[:2]
    if corner_center_x >= (image_width / 2) - (image_width * margin_ratio) and corner_center_x <= (image_width / 2) + (image_width * margin_ratio):
        if corner_center_y >= (image_height / 2) - (image_height * margin_ratio) and corner_center_y <= (image_height / 2) + (image_height * margin_ratio):
            return True
    return False


    
def my_estimatePoseSingleMarkers(corner, marker_size, mtx, distortion):
    '''
    https://stackoverflow.com/questions/76802576/how-to-estimate-pose-of-single-marker-in-opencv-python-4-8-0
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []

    nada, R, t = cv2.solvePnP(marker_points, corner, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
    rvecs.append(R)
    tvecs.append(t)
    trash.append(nada)
    return rvecs, tvecs, trash

def main(args=None):
    rclpy.init(args=args)
    node = nAruco()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()