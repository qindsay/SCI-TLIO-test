## Introduction
Here's a structured Python implementation for offline processing, fusing IMU data and external camera-based AprilTag detection using an Extended Kalman Filter (EKF). You can adapt this framework easily to your recorded IMU and RGB data.

- Detects an AprilTag marker using a static RGB camera (OpenCV).
- Fuses IMU readings and camera pose detections via a simple EKF, correcting drift and improving accuracy.
- Outputs the robot’s estimated trajectory for visualization.

## Future Directions
- Explore deep learning-based object detection (e.g., YOLO) if AprilTag markers are impractical, though this increases computational demands.
- Replace the simple EKF with available library: https://filterpy.readthedocs.io/en/latest/
- Consider graph-based optimization instead of EKF for potentially higher accuracy, using libraries like g2o or Ceres Solver.


## Steps 
1. Install dependencies: `pip install numpy opencv-python apriltag matplotlib`

2. Setup Hardware: 
    - Place an AprilTag marker on the robot for easy detection.
    - Mount the RGB camera on a wall or pole with a clear view of the robot’s movement area, and lighting conditions should be consistent to aid detection.
    - Ensure the IMU is securely attached to the robot.

3. Collect Data: Record video from the camera and IMU data (accelerometer and gyroscope readings) with synchronized timestamps.

4. Sample "imu_data.csv" => I don't know your IMU, so you should base on your IMU and try to extract data same as below.
```
time, ax, ay, az
0.00, 0.01, -0.02, 9.81
0.01, 0.02, -0.01, 9.81
...
```

5. Camera Calibration:

- You can print chessboard pattern from this link (https://calib.io/pages/camera-calibration-pattern-generator) and fix it to a flat surface.
- Load and Detect Chessboard Corners:

```python
import cv2
import numpy as np
import glob

# Define the chessboard size
chessboard_size = (9, 6)  # inner corners (width, height)

# Prepare object points (0,0,0), (1,0,0), ..., (8,5,0)
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Arrays to store object points and image points
objpoints = []  # 3D real world points
imgpoints = []  # 2D image points

# Load calibration images
images = glob.glob('calib_images/*.jpg')  # path to your calibration images

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        # Optional: draw and display the corners
        img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()


# Calibrate the camera
ret, camera_matrix, camera_dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# Print the results
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", camera_dist)

```

6. Python sample code:

```python
import numpy as np
import cv2
import apriltag
import matplotlib.pyplot as plt

# Simple EKF Implementation
class EKF:
    def __init__(self):
        self.dt = 0.01  # time step (100 Hz IMU assumed) => Adjust IMU sampling rate (dt) and timestamps according to your actual recorded IMU data. You should ensure synchronization between IMU and video timestamps (e.g., via linear interpolation if needed).
        self.x = np.zeros((6, 1))  # [x, y, z, vx, vy, vz]
        self.P = np.eye(6) * 0.1
        self.Q = np.eye(6) * 0.01
        self.R = np.eye(3) * 0.05
        self.F = np.eye(6)
        for i in range(3):
            self.F[i, i+3] = self.dt
        self.H = np.hstack((np.eye(3), np.zeros((3, 3))))

    def predict(self, accel):
        # Acceleration as control input
        u = accel.reshape((3, 1))

        B = np.vstack((0.5*self.dt**2 * np.eye(3), self.dt * np.eye(3)))
        self.x = self.F @ self.x + B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        z = z.reshape((3,1))
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def get_state(self):
        return self.x[:3].flatten()

# AprilTag Detector Class
class AprilTagDetector:
    def __init__(self, tag_size, camera_matrix, camera_dist):
        self.detector = apriltag.Detector()
        self.tag_size = tag_size
        self.camera_matrix = camera_matrix
        self.camera_dist = camera_dist

    def detect_pose(self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        if results:
            r = results[0]  # assuming one tag on robot
            corners = r.corners
            obj_pts = np.array([
                [-self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2,  self.tag_size/2, 0],
                [-self.tag_size/2,  self.tag_size/2, 0]
            ])
            ret, rvec, tvec = cv2.solvePnP(
                obj_pts, corners,
                self.camera_matrix,
                self.camera_dist,
                flags=cv2.SOLVEPNP_ITERATIVE
            )
            if ret:
                return tvec.flatten()
        return None

# Example usage:
def main():
    # Example camera calibration parameters => Replace camera_matrix and camera_dist with your camera's actual intrinsic calibration.
    camera_matrix = np.array([[600, 0, 320],
                              [0, 600, 240],
                              [0, 0, 1]])
    camera_dist = np.zeros(5)
    tag_size = 0.1  # 10cm marker

    detector = AprilTagDetector(tag_size, camera_matrix, camera_dist)
    ekf = EKF()

    trajectory = []
    imu_data = np.loadtxt('imu_data.csv', delimiter=',')  # [time, ax, ay, az]
    timestamps = imu_data[:, 0]
    accels = imu_data[:, 1:4]

    cap = cv2.VideoCapture('video.mp4')
    frame_rate = cap.get(cv2.CAP_PROP_FPS)
    video_frame_idx = 0

    for idx, t in enumerate(timestamps):
        accel = accels[idx]
        ekf.predict(accel)

        # Match video frame to IMU timestamp
        expected_frame_idx = int(t * frame_rate)
        while video_frame_idx <= expected_frame_idx:
            ret, frame = cap.read()
            if not ret:
                break
            video_frame_idx += 1

        if ret:
            pose = detector.detect_pose(frame)
            if pose is not None:
                ekf.update(pose)

        trajectory.append(ekf.get_state())

    cap.release()
    trajectory = np.array(trajectory)
    
    # Save trajectory
    np.savetxt('trajectory.csv', np.array(trajectory), delimiter=',')

    # Plot trajectory of the robot car’s estimated path
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2], '-b', label='EKF Trajectory')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')
    ax.legend()
    plt.show()

if __name__ == '__main__':
    main()
```
