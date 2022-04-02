import math
import time

COUNTER_VALUE = 2


class States(object):
    def __init__(self, telloCV):
        self.telloCV = telloCV
        self.drone = telloCV.drone
        self.current = self.search_for_aruco
        self.pids = telloCV.pids
        self.arucoIDs = [4, 2, 3]
        self.arucoID = self.arucoIDs[0]
        self.counter = COUNTER_VALUE
        self.timer = None
        self.aruco_count = 0

    def search_for_aruco(self, image):
        self.drone.set_yaw(0.3)
        detected, coord, angles = self.telloCV.arucoDetector.detectAruco(image, self.arucoID)
        if detected:
            self.drone.set_yaw(0.0)
            self.current = self.follow_aruco

    def follow_aruco(self, image):
        detected, coord, angles = self.telloCV.arucoDetector.detectAruco(image, self.arucoID)
        if detected:
            x, y, z = coord
            aruco_roll_angle = angles[1]
            yaw_angle = math.atan(x / z)
            control_yaw = self.pids.pid_yaw(yaw_angle)
            control_pitch = self.pids.pid_pitch(z)
            control_height = self.pids.pid_height(y)
            control_roll = self.pids.pid_roll(aruco_roll_angle)

            self.drone.set_yaw(- control_yaw)
            self.drone.set_pitch(- control_pitch)
            self.drone.set_throttle(control_height)
            self.drone.set_roll(- control_roll)
            if self.isArucoSet(y, z, aruco_roll_angle, yaw_angle):
                self.drone.set_yaw(0)
                self.drone.set_pitch(0)
                self.drone.set_throttle(0)
                self.drone.set_roll(0)
                self.timer = time.time()
                self.current = self.go_through_circle

        else:
            self.drone.set_yaw(0)
            self.drone.set_pitch(0)
            self.drone.set_throttle(0)
            self.drone.set_roll(0)

            self.current = self.search_for_aruco

    def go_through_circle(self, image):
        self.drone.set_yaw(0)
        self.drone.set_pitch(0.4)
        self.drone.set_throttle(0)
        self.drone.set_roll(0)
        if time.time() > self.timer + 2.3:
            self.drone.set_pitch(0)
            self.aruco_count += 1
            try:
                self.arucoID = self.arucoIDs[self.aruco_count]
            except IndexError:
                self.drone.set_yaw(0)
                self.drone.set_pitch(0)
                self.drone.set_throttle(0)
                self.drone.set_roll(0)
                self.drone.land()
                self.telloCV.auto_control = False
            self.current = self.search_for_aruco

    def isArucoSet(self, y, z, roll_angle, yaw_angle):
        pids = self.pids
        if (abs(yaw_angle - pids.pid_yaw.setpoint) < 0.08 and
                abs(z - pids.pid_pitch.setpoint) < 0.04 and
                abs(y - pids.pid_height.setpoint) < 0.04 and
                abs(roll_angle - pids.pid_roll.setpoint) < 0.08):
            self.counter -= 1
        if self.counter == 0:
            self.counter = COUNTER_VALUE
            return True
        else:
            return False
