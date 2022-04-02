from simple_pid import PID

I_UP = 1
I_DOWN = 2
P_UP = 3
P_DOWN = 4
D_UP = 5
D_DOWN = 6
SP_UP = 7
SP_DOWN = 8

PID_BASE = 0.01


class PIDs(object):
    def __init__(self):
        self.pid_yaw = PID(1, 0, 0.2, setpoint=0.0, output_limits=(-1, 1))
        self.pid_pitch = PID(1, 0, 0.2, setpoint=1.00, output_limits=(-0.3, 0.3))
        self.pid_roll = PID(0.5, 0, 0.2, setpoint=0.0, output_limits=(-0.6, 0.6))
        self.pid_height = PID(1, 0, 0.2, setpoint=0.2, output_limits=(-0.5, 0.5))
        self.pids = [self.pid_yaw, self.pid_pitch, self.pid_roll, self.pid_height]
        self.pid_id = 0
        self.init_pid_controls()

    def init_pid_controls(self):
        self.pid_controls = {
            '1': lambda: self.changePidId(1),
            '2': lambda: self.changePidId(2),
            '3': lambda: self.changePidId(3),
            ',': lambda: self.changePidId(4),
            '<65439>': lambda: self.changePidId(4),  # This is what you get when you press "," on the right side
            '7': lambda: self.changePIDValue(P_UP),  # of the keyboard
            '4': lambda: self.changePIDValue(P_DOWN),
            '8': lambda: self.changePIDValue(I_UP),
            '5': lambda: self.changePIDValue(I_DOWN),
            '<65437>': lambda: self.changePIDValue(I_DOWN),  # This is what you get when you press "5" on the right
            '9': lambda: self.changePIDValue(D_UP),          # side of the keyboard
            '6': lambda: self.changePIDValue(D_DOWN),
            '/': lambda: self.changePIDValue(SP_DOWN),
            '*': lambda: self.changePIDValue(SP_UP),
        }

    def changePidId(self, id):
        self.pid_id = id - 1

    def changePIDValue(self, pid_value):
        if pid_value == 0:
            return
        pid = self.pids[self.pid_id]
        print("Changing PID number {}.".format(self.pid_id))
        if pid_value == I_UP:
            pid.Ki += PID_BASE
        if pid_value == I_DOWN:
            pid.Ki -= PID_BASE
        if pid_value == P_UP:
            pid.Kp += PID_BASE
        if pid_value == P_DOWN:
            pid.Kp -= PID_BASE
        if pid_value == D_UP:
            pid.Kd += PID_BASE
        if pid_value == D_DOWN:
            pid.Kd -= PID_BASE
        if pid_value == SP_UP:
            pid.setpoint += PID_BASE
        if pid_value == SP_DOWN:
            pid.setpoint -= PID_BASE
        print("New PID values: P: {} I: {} D: {} Setpoint: {}".format(pid.Kp, pid.Ki, pid.Kd, pid.setpoint))
