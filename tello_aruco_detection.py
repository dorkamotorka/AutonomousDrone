import numpy
from cv2 import aruco
from scipy.spatial.transform import Rotation as R

MAX_NUMBER_OF_ARUCOS = 20


class ArucoDetector(object):
    def __init__(self):
        self.cam_mtx = numpy.genfromtxt("./Calibration/CAM_MTX", delimiter=',')
        self.dist = numpy.genfromtxt("./Calibration/DIST", delimiter=',')

        self.paramArUco = aruco.DetectorParameters_create()
        self.markerDict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        self.arucoXYZ = [[None, None, None] for i in
                         range(MAX_NUMBER_OF_ARUCOS)]
        self.arucoAngles = [[None, None, None] for i in
                            range(MAX_NUMBER_OF_ARUCOS)]
        self.arucoDetected = [False for i in range(MAX_NUMBER_OF_ARUCOS)]

        self.rvecs = None
        self.tvecs = None
        self.markerCorners = None
        self.markerIDs = None
        self.rejectedCandidates = None
        self._objPoints = None

    def detectAruco(self, image, arucoID):
        self.markerCorners, self.markerIDs, self.rejectedCandidates = aruco.detectMarkers(
            image, self.markerDict, parameters=self.paramArUco)

        self.rvecs, self.tvecs, self._objPoints = aruco.estimatePoseSingleMarkers(
            self.markerCorners, 0.093, self.cam_mtx, self.dist)

        # First clear the arrays
        self.arucoXYZ = [[None, None, None] for i in
                         range(MAX_NUMBER_OF_ARUCOS)]
        self.arucoAngles = [[None, None, None] for i in
                            range(MAX_NUMBER_OF_ARUCOS)]
        self.arucoDetected = [False for i in range(MAX_NUMBER_OF_ARUCOS)]

        # Write in the detected arucos
        if self.tvecs is not None:
            for id, _ in enumerate(self.tvecs):
                try:
                    self.arucoXYZ[self.markerIDs[id][0]] = self.tvecs[id][0]
                    self.arucoDetected[self.markerIDs[id][0]] = True
                    self.arucoAngles[self.markerIDs[id][0]] = R.from_rotvec(self.rvecs[id][0]).as_euler('zyx')
                except IndexError:
                    print("Marker ID out of range!")

        return self.arucoDetected[arucoID], self.arucoXYZ[arucoID], self.arucoAngles[arucoID]

    def drawLastMarkers(self, image):
        if self.rvecs is not None:
            image = aruco.drawDetectedMarkers(image, self.markerCorners,
                                              self.markerIDs)
            for id, _ in enumerate(self.rvecs):
                image = aruco.drawAxis(image, self.cam_mtx, self.dist,
                                       self.rvecs[id], self.tvecs[id], 0.05)
