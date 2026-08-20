import cv2
import cv2.aruco as aruco
import numpy as np

image = cv2.imread("/home/user/amr/amr_lv4_ws/bitmap.jpg")

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

arucoDictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters_create()

corners, ids, rejectedCandidates = aruco.detectMarkers(
    gray, arucoDictionary, parameters=parameters
)

cameraMatrix = np.array([[600.0,   0.0, 320.0],
                          [  0.0, 600.0, 240.0],
                          [  0.0,   0.0,   1.0]], dtype=np.float32)
distortionCoefficients = np.zeros((5, 1), dtype=np.float32)
markerLength = 0.05

if ids is not None:
    rvecs, tvecs, objectPoints = aruco.estimatePoseSingleMarkers(
        corners, markerLength, cameraMatrix, distortionCoefficients
    )
    for rvec, tvec, id in zip(rvecs, tvecs, ids):
        rotationMatrix, rotationJacobian = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = rotationMatrix
        T[:3, 3] = tvec[0]
        print(f"Marker ID: {id[0]}")
        print("T_marker_camera:\n", T)
