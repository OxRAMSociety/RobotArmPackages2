#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

import cv2
import numpy as np

# Takes locations of the inner corners, and the board size and calculates all corners
def compute_grid_positions(corners, board_size):
    grid_positions = np.zeros(
        (board_size[0] + 2, board_size[1] + 2, 2), dtype=np.float32
    )
    # Inner corners
    for i in range(1, board_size[0] + 1):
        for j in range(1, board_size[1] + 1):
            grid_positions[i, j] = corners[(j - 1) + (i - 1) * board_size[1]].ravel()
    return grid_positions


class ChessboardPublisher(Node):
    def __init__(self):
        # Create a publisher
        super().__init__("chessboard_publisher")

        # Parameters which can be passed to the node
        # Topic to listen for camera events on
        self.declare_parameter("camera_topic", "/image_raw/compressed")
        self.camera_topic = self.get_parameter("camera_topic").value

        self.chessboard_publisher = self.create_publisher(Image, "chessboard_topic", 10)

        # Create an instance of CvBridge
        self.cv_bridge = CvBridge()
        # Subscribe to the video
        self.subscription = self.create_subscription(
            CompressedImage, self.camera_topic, self.on_new_camera_frame, 10
        )

    # Turns image gray scale, finds chessboard corners and draws on the corners
    # Logs: Chessboard detected, type of corners, grid positions
    def process_image(self, cv_image):
        logger = self.get_logger()

        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        ret = None

        # successful_board_sizes = []

        possible = []
        for i in range(3, 8):
            for j in range(i, 8):
                possible.append((i,j))
        possible.sort(key = lambda x:x[0] * x[1], reverse = True)

        for board_size in possible:
            # Chessboard size in squares(width and height)
            # board_size = (i, j)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, board_size, None)  # MatLike
            
            if ret:
                break
                # successful_board_sizes.append(board_size)

        # board_size = max(successful_board_sizes, key = lambda x: x[0] * x[1])
        # ret, corners = cv2.findChessboardCorners(gray, board_size, None)
        # logger.info(f"Successful board sizes {successful_board_sizes}")

        # If the chessboard is detected, draw the grid and compute the grid positions
        if ret:
            logger.info("Chessboard corners detected.")
            logger.info(str(type(corners)))

            # Improve corner accuracy
            win_size = (3, 3)
            zero_zone = (-1, -1)
            criteria = (cv2.TERM_CRITERIA_EPS, 0, 0.00001)

            corners2 = cv2.cornerSubPix(
                gray, corners, win_size, zero_zone, criteria
            )

            # Find other corners
            corners3 = compute_grid_positions(corners2, board_size)

            # Object points: 3D coordinates of the internal corners (x, y, 0)
            objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
            
            # Read Camera Matrix and Distortion Coefficients from .yaml file
            # Currently hardcoded - TODO: READ IT FROM FILE
            mtx = np.array([
                [796.62207,   0.     , 674.57298], 
                [0.     , 856.057  , 586.88776, ], 
                [0.     ,   0.     ,   1.     ]
                ])
            dist = np.array([-0.274258, 0.040343, -0.054555, -0.014826, 0.000000])
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([objp], [corners2], gray.shape[::-1], None, None)


            # Find the rotation and translation vectors
            # Depth cannot be determined without another camera
            ret, rvecs, tvecs = cv2.solvePnP(objp, corners2, mtx, dist)
            logger.info(f'rvecs: {rvecs.ravel()} tvecs: {tvecs.ravel()}')

            rotM = cv2.Rodrigues(rvecs)[0]
            cameraPosition = -np.matrix(rotM).T * np.matrix(tvecs)
            logger.info(f'cam pos {cameraPosition.ravel()}')


            #######
            ## This section of code takes points in 3D and draws them on the image plane
            def tupleint32(it):
                return tuple(np.int32(it))
            
            if True:
                def draw(img, corners, imgpts):
                    corner = tupleint32(corners[0].ravel())
                    img = cv2.line(img, corner, tupleint32(imgpts[0].ravel()), (255, 0, 0), 2)
                    img = cv2.line(img, corner, tupleint32(imgpts[1].ravel()), (0, 255, 0), 2)
                    img = cv2.line(img, corner, tupleint32(imgpts[2].ravel()), (0, 0, 255), 2)
                    return img

                axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, 3]]).reshape(-1, 3)
            else:
                def draw(img, corners, imgpts):
                    imgpts = np.int32(imgpts).reshape(-1, 2)
                    # draw ground floor in green
                    img = cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), -3)
                    # draw pillars in blue color
                    for i, j in zip(range(4), range(4, 8)):
                        img = cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255), 1)
                    # draw top layer in red color
                    img = cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 1)
                    return img

                w, b, h = (4, 3, 1)
                axis = np.float32([[0, 0, 0], [0, b, 0], [w, b, 0], [w, 0, 0], [0, 0, -h], [0, b, -1], [w, b, -h], [w, 0, -h] ])

            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, mtx, dist)
            draw(cv_image, corners2, imgpts)

            #########

            # Draw on circle
            for i in range(0, board_size[0] + 2):
                for j in range(0, board_size[1] + 2):
                    cv2.circle(
                        cv_image, 
                        (int(corners3[i, j][0]), int(corners3[i, j][1])), 
                        10, 
                        (255, 0, 0), 
                        10, 
                    )
            logger.info(str(type(corners3)))
            # Print the grid positions
            logger.info(f'board size: {board_size}')
            logger.info("Grid positions:")
            # logger.info(corners)
        else:
            logger.info("Chessboard corners not detected.")
        return cv_image

    def on_new_camera_frame(self, msg):
        frame = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        assert frame is not None, "Error reading frame"
        frame = self.process_image(frame)
        assert frame is not None, "Error processing frame"

        img_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        self.chessboard_publisher.publish(img_msg)

        # waitKey is a workaround - allows OpenCV to display result of imshow
        cv2.imshow("GRID1", frame)
        cv2.waitKey(1)

        # TODO: run video.release() when exiting


def main(args=None):
    # Initiate the node
    rclpy.init(args=args)
    publisher_node = ChessboardPublisher()

    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
