import argparse
import time
from servo import COMM_SUCCESS, Port, ServoConnection
from servos import SCS0009, STS3215

parser = argparse.ArgumentParser(description="Move")
parser.add_argument("port", type=str, nargs="?", help="Port")
args = parser.parse_args()

port = Port(args.port)


class Servo:
    def __init__(self, id, name, conn: STS3215 | SCS0009, reverse: bool):
        self.id = id
        self.name = name
        self.conn = conn
        self.reverse = reverse

        _, res, _ = conn.ping(id)
        if res != COMM_SUCCESS:
            raise Exception(f"Servo {id} not connected")

        self.min, _, _ = conn.read(id, conn.MIN_POSITION_LIMIT)
        self.max, _, _ = conn.read(id, conn.MAX_POSITION_LIMIT)

    def move(self, pos: float):
        relative = abs(self.min - self.max) * pos
        pos = (self.max - relative) if self.reverse else (self.min + relative)
        self.conn.write(self.id, self.conn.GOAL_POSITION, int(pos))

    def start(self):
        self.conn.write(self.id, self.conn.TORQUE_ENABLE, 1)

    def stop(self):
        self.conn.write(self.id, self.conn.TORQUE_ENABLE, 0)

    def __repr__(self):
        return f"Servo(id={self.id}, name='{self.name}', type={self.type.__name__}, range=({self.min} → {self.max}), reverse={self.reverse})"


class Arm:
    def __init__(self, side, port: ServoConnection):
        self.side = side
        start = 30 if self.side == "right" else 40

        reverse = side == "right"
        scs = SCS0009(port)
        sts = STS3215(port)
        self.thumb_rotation = Servo(start + 1, "thumb_rotation", scs, reverse)
        self.thumb = Servo(start + 2, "thumb", scs, reverse)
        self.index = Servo(start + 3, "index", scs, reverse)
        self.middle = Servo(start + 4, "middle", scs, not reverse)
        self.ring = Servo(start + 5, "ring", scs, reverse)
        self.little = Servo(start + 6, "little", scs, not reverse)
        self.servos = [
            self.thumb_rotation,
            self.thumb,
            self.index,
            self.middle,
            self.ring,
            self.little,
        ]

    def start(self):
        for servo in self.servos:
            servo.start()

    def stop(self):
        for servo in self.servos:
            servo.stop()


left = Arm("left", port)

for hand in [left]:

    hand.start()

    pos = 0
    hand.thumb_rotation.move(pos)
    hand.thumb.move(pos)
    hand.index.move(pos)
    hand.middle.move(pos)
    hand.ring.move(pos)
    hand.little.move(pos)
    time.sleep(3)
    hand.stop()
