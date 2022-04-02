#!/usr/bin/env python3
import signal
import time

import av
import cv2
import numpy
import tello_drone_extended
from pynput import keyboard

from tello_aruco_detection import ArucoDetector
from tello_pids import PIDs
from tello_states import States

running = True


def main():
    tello = AutoTello()
    signal.signal(signal.SIGINT, tello.exit_program)
    frame_skip = 300
    for packet in tello.container.demux((tello.vid_stream,)):
        for frame in packet.decode():
            if 0 < frame_skip:
                frame_skip -= 1
                continue
            timeBeforeProcess = time.time()
            image = tello.process_frame(frame)
            cv2.imshow('tello', image)
            _ = cv2.waitKey(1) & 0xFF
            if frame.time_base < 1.0 / 60:
                time_base = 1.0 / 60
            else:
                time_base = frame.time_base
            frame_skip = int((time.time() - timeBeforeProcess) / time_base)
            if not running:
                exit()

    cv2.destroyAllWindows()
    print("Ending stream, exiting\n")


class AutoTello(object):
    """
    The main class for tello auto controls
    """

    def __init__(self):
        self.prev_flight_data = None
        self.auto_control = False
        self.keydown = False

        self.date_fmt = '%Y-%m-%d_%H%M%S'
        self.speed = 50
        self.drone = tello_drone_extended.TelloExtended()
        self.arucoDetector = ArucoDetector()
        self.pids = PIDs()
        self.states = States(self)
        self.init_drone()
        self.init_controls()

        self.container = av.open(self.drone.get_video_stream())
        self.vid_stream = self.container.streams.video[0]

    def init_drone(self):
        """Connect, uneable streaming and subscribe to events"""
        # self.drone.log.set_level(2)
        self.drone.connect()
        self.drone.start_video()
        self.drone.subscribe(self.drone.EVENT_FLIGHT_DATA,
                             self.flight_data_handler)
        self.drone.set_alt_limit(3)

    def init_controls(self):
        """Define keys and add listener"""
        self.controls = {
            'w': 'forward',
            's': 'backward',
            'a': 'left',
            'd': 'right',
            'Key.space': 'up',
            'Key.shift': 'down',
            'Key.shift_r': 'down',
            'q': 'counter_clockwise',
            'e': 'clockwise',
            'i': lambda speed: self.drone.flip_forward(),
            'k': lambda speed: self.drone.flip_back(),
            'j': lambda speed: self.drone.flip_left(),
            'l': lambda speed: self.drone.flip_right(),
            # arrow keys for fast turns and altitude adjustments
            'Key.left': lambda speed: self.drone.counter_clockwise(speed),
            'Key.right': lambda speed: self.drone.clockwise(speed),
            'Key.up': lambda speed: self.drone.up(speed),
            'Key.down': lambda speed: self.drone.down(speed),
            'Key.f5': lambda speed: self.drone.takeoff(),
            'Key.backspace': lambda speed: self.drone.land(),
            'p': lambda speed: self.palm_land(speed),
            't': lambda speed: self.toggle_auto_control(speed),
        }
        self.key_listener = keyboard.Listener(on_press=self.on_press,
                                              on_release=self.on_release)
        self.key_listener.daemon = True  # Exit the program nicely
        self.key_listener.start()

    def on_press(self, keyname):
        """handler for keyboard listener"""
        if self.keydown:
            return
        try:
            self.keydown = True
            keyname = str(keyname).strip('\'')
            print('+' + keyname)
            if keyname == 'Key.esc':
                self.drone.quit()
                exit(0)
            if keyname in self.controls:
                key_handler = self.controls[keyname]
                if isinstance(key_handler, str):
                    getattr(self.drone, key_handler)(self.speed)
                else:
                    key_handler(self.speed)
            elif keyname in self.pids.pid_controls:
                key_handler = self.pids.pid_controls[keyname]
                key_handler()

        except AttributeError:
            print('special key {0} pressed'.format(keyname))

    def on_release(self, keyname):
        """Reset on key up from keyboard listener"""
        self.keydown = False
        keyname = str(keyname).strip('\'')
        print('-' + keyname)
        if keyname in self.controls:
            key_handler = self.controls[keyname]
            if isinstance(key_handler, str):
                getattr(self.drone, key_handler)(0)
            else:
                key_handler(0)

    def toggle_auto_control(self, speed):
        """ Handle auto_control keypress"""
        if speed == 0:  # handle key up event
            return
        self.auto_control = not self.auto_control
        print("auto_control:", self.auto_control)
        return

    def write_hud(self, frame):
        """Draw drone info, auto_control and record on frame"""
        borderType = cv2.BORDER_CONSTANT
        left = int(0.50 * frame.shape[1])
        frame = cv2.copyMakeBorder(frame, 0, 0, left, 0, borderType, None,
                                   [0, 0, 0])
        stats = self.prev_flight_data.split('|')

        if self.auto_control:
            stats.append("AUTO CONTROL: ON")
        else:
            stats.append("AUTO CONTROL: OFF")
        stats.append(f"STATE: {self.states.current.__name__}")
        pids = self.pids
        stats.append("Yaw:")
        stats.append(f"Sp: {pids.pid_yaw.setpoint:.2f}")
        stats.append(f"P:{pids.pid_yaw.Kp:.2f} I:{pids.pid_yaw.Ki:.2f} D:{pids.pid_yaw.Kd:.2f}")
        stats.append("Pitch:")
        stats.append(f"Sp: {pids.pid_pitch.setpoint:.2f}")
        stats.append(f"P:{pids.pid_pitch.Kp:.2f} I:{pids.pid_pitch.Ki:.2f} D:{pids.pid_pitch.Kd:.2f}")
        stats.append("Roll:")
        stats.append(f"Sp: {pids.pid_roll.setpoint:.2f}")
        stats.append(f"P:{pids.pid_roll.Kp:.2f} I:{pids.pid_roll.Ki:.2f} D:{pids.pid_roll.Kd:.2f}")
        stats.append("Height:")
        stats.append(f"Sp: {pids.pid_height.setpoint:.2f}")
        stats.append(f"P:{pids.pid_height.Kp:.2f} I:{pids.pid_height.Ki:.2f} D:{pids.pid_height.Kd:.2f}")
        for idx, stat in enumerate(stats):
            text = stat.lstrip()
            cv2.putText(frame, text, (0, 30 + (idx * 30)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 255, 255), lineType=30)
        return frame

    def process_frame(self, frame):
        """convert frame to cv2 image and show"""
        image = cv2.cvtColor(numpy.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        if self.auto_control:
            self.states.current(image)
        else:
            self.arucoDetector.detectAruco(image, 0)
        self.arucoDetector.drawLastMarkers(image)
        image = self.write_hud(image)
        return image

    def flight_data_handler(self, event, sender, data):
        """Listener to flight data from the drone."""
        text = str(data)
        if self.prev_flight_data != text:
            self.prev_flight_data = text

    def palm_land(self, speed):
        """Tell drone to land"""
        if speed == 0:
            return
        self.drone.palm_land()

    def exit_program(self, *args):
        global running
        print("Exiting the program!")
        self.drone.quit()  # Apparently doesn't always work (:
        running = False


if __name__ == '__main__':
    main()
