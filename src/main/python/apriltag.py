import numpy as np
import cv2
import matplotlib.pyplot as plt
import os

# basically fixes the intrinsic parameters and is the class that returns the 3D stuff
# printed 3dpose --> tvec (x: left/right, y: up/down, z: front/back), rvec
# max z is 20 feet (detects, but not necessarily accurate); max x is 1 foot on either side
# at 18 in -> max left/right was 4.5 in
class AprilTag():

    def __init__(self):
        self.camera_matrix = np.load('calibration_data/camera2_matrix.npy')
        self.dist_coeffs = np.load('calibration_data/camera2_dist.npy')

    def calibrate(self, dirpath, square_size, width, height, visualize=False):
        """ Apply camera calibration operation for images in the given directory path. """

        # termination criteria
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,6,0)
        objp = np.zeros((height*width, 3), np.float32)
        objp[:, :2] = np.mgrid[0:width, 0:height].T.reshape(-1, 2)

        objp = objp * square_size

        # Arrays to store object points and image points from all the images.
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        images = os.listdir(dirpath)
        for fname in images:
            img = cv2.imread(os.path.join(dirpath, fname))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (width, height), None)

            # If found, add object points, image points (after refining them)
            if ret:
                objpoints.append(objp)

                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (width, height), corners2, ret)

            if visualize:
                cv2.imshow('img',img)
                cv2.waitKey(0)

        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
        self.camera_matrix = mtx
        self.dist_coeffs = dist

        np.save('calibration_data/camera2_matrix.npy',mtx)
        np.save('calibration_data/camera2_dist.npy',dist)

    def draw_axis_on_image(self, image, camera_matrix, dist_coeffs, rvec, tvec, size=1):
        try:
            # Define axis length
            length = size

            # 3D axis points in the marker coordinate system
            axis_points_3d = np.float32([[0, 0, 0], [length, 0, 0], [0, length, 0], [0, 0, -length]])

            # Project 3D points to image plane
            axis_points_2d, _ = cv2.projectPoints(axis_points_3d, rvec, tvec, camera_matrix, dist_coeffs)

            # Convert to integer
            axis_points_2d = np.int32(axis_points_2d).reshape(-1, 2)

            # Draw axis lines directly on the image
            cv2.line(image, tuple(axis_points_2d[0]), tuple(axis_points_2d[1]), (0, 0, 255), 2)  # X-axis (red)
            cv2.line(image, tuple(axis_points_2d[0]), tuple(axis_points_2d[2]), (0, 255, 0), 2)  # Y-axis (green)
            cv2.line(image, tuple(axis_points_2d[0]), tuple(axis_points_2d[3]), (255, 0, 0), 2)  # Z-axis (blue)

            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.4
            font_thickness = 1
            text_color = (255, 0, 255)  # White color
            text_position = (10, 30)  # Top-left corner coordinates
            # Add text to the image
            cv2.putText(image, str(tvec), text_position, font, font_scale, text_color, font_thickness)


            return image

        except Exception as e:
            print(f"An error occurred: {e}")
            return None

    def estimate_pose_single_marker(self, corners, marker_size, camera_matrix, dist_coeffs):
        try:
            # Ensure corners is a NumPy array
            corners = np.array(corners)
            # Define the 3D coordinates of the marker corners in the marker coordinate system
            #marker_points_3d = np.array([[0, 0, 0], [marker_size, 0, 0], [marker_size, marker_size, 0], [0, marker_size, 0]], dtype=np.float32)
            marker_points_3d = np.array([[-marker_size / 2, -marker_size / 2, 0], [marker_size / 2, -marker_size / 2, 0], [marker_size / 2, marker_size / 2, 0], [-marker_size / 2, marker_size / 2, 0]], dtype=np.float32)

            # Reshape the corners to a flat array
            image_points_2d = corners.reshape(-1, 2)

            # Convert image points to float32
            image_points_2d = np.float32(image_points_2d)

            # Solve PnP problem to estimate pose
            _, rvec, tvec = cv2.solvePnP(marker_points_3d, image_points_2d, camera_matrix, dist_coeffs)
            
            return rvec, tvec * 39.3701

        except Exception as e:
            print(f"An error occurred: {e}")
            return None, None
        
    def estimate_3d_pose(self, image, frame_ann):
            ARUCO_LENGTH_METERS = 0.1524

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Create an AprilTag detector
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
            detector = cv2.aruco.ArucoDetector(aruco_dict)
            # Detect AprilTags in the image
            corners, ids, rejected_img_points = detector.detectMarkers(gray)
            pose_data = {}
            num_tags = len(ids) if ids is not None else 0
            #print(str(num_tags) + ' AprilTags detected')
            if num_tags != 0:
                # Draw the detected markers on the image
                cv2.aruco.drawDetectedMarkers(image, corners, ids)

                # Estimate the pose of each detected marker
                for i in range(len(ids)):
                    # Estimate the pose
                    rvec, tvec= self.estimate_pose_single_marker(corners[i], ARUCO_LENGTH_METERS, self.camera_matrix, self.dist_coeffs)
                    pose_data[ids[i][0]] = (tvec, rvec)
                    # Draw the 3D pose axis on the image
                    self.draw_axis_on_image(frame_ann, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

                # Display the result
                cv2.imshow('AprilTag Pose Estimation', image)
            else:
                #print("No AprilTags detected in the image.")
                pass
            return pose_data
            



