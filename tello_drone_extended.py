import tellopy


class TelloExtended(tellopy.Tello):
    def __init__(self):
        tellopy.Tello.__init__(self)

    def go_forward_cm(self, cm):
        command = f"forward {cm}"
        self.sock.sendto(command.encode('utf-8'), self.tello_addr)

    def go_to(self, x, y, z, speed):
        command = f"go {x} {y} {z} {speed}"
        self.sock.sendto(command.encode('utf-8'), self.tello_addr)

    def go_to_curve(self, x1, y1, z1, x2, y2, z2, speed):
        command = f"curve {x1} {y1} {z1} {x2} {y2} {z2} {speed}"
        self.sock.sendto(command.encode('utf-8'), self.tello_addr)

