import argparse
import time
from servo import COMM_SUCCESS, ServoConnection
from servos import SCS0009, STS3215

parser = argparse.ArgumentParser(description="Move")
parser.add_argument("port", type=str, nargs="?", help="Port")
parser.add_argument("new_id", type=int, nargs="?", help="New servo ID")
args = parser.parse_args()

conn = ServoConnection(args.port)


hand_servos = {
    "thumb_rotation": 1,
    "thumb": 2,
    "index": 3,
    "middle": 4,
    "ring": 5,
    "little": 6,
}


class Servo:
    def __init__(
        self, id, name, conn: ServoConnection, type: STS3215 | SCS0009, reverse: bool
    ):
        self.id = id
        self.name = name
        self.conn = conn
        self.type = type
        self.reverse = reverse

        _, res, _ = conn.ping(id, type)
        if res != COMM_SUCCESS:
            raise Exception(f"Servo {id} not connected")

        self.min, _, _ = conn.read(id, type.MIN_POSITION_LIMIT)
        self.max, _, _ = conn.read(id, type.MAX_POSITION_LIMIT)

    def move(self, pos: float):
        relative = abs(self.min - self.max) * pos
        pos = (self.max - relative) if self.reverse else (self.min + relative)
        self.conn.write(self.id, self.type.GOAL_POSITION, int(pos))

    def start(self):
        self.conn.write(self.id, self.type.TORQUE_ENABLE, 1)

    def stop(self):
        self.conn.write(self.id, self.type.TORQUE_ENABLE, 0)

    def __repr__(self):
        return f"Servo(id={self.id}, name='{self.name}', type={self.type.__name__}, range=({self.min} → {self.max}), reverse={self.reverse})"


class Arm:
    def __init__(self, side, conn: ServoConnection):
        self.side = side
        start = 30 if self.side == "right" else 40
        self.conn = conn

        reverse = side == "right"
        self.thumb_rotation = Servo(start + 1, "thumb_rotation", conn, SCS0009, reverse)
        self.thumb = Servo(start + 2, "thumb", conn, SCS0009, reverse)
        self.index = Servo(start + 3, "index", conn, SCS0009, reverse)
        self.middle = Servo(start + 4, "middle", conn, SCS0009, not reverse)
        self.ring = Servo(start + 5, "ring", conn, SCS0009, reverse)
        self.little = Servo(start + 6, "little", conn, SCS0009, not reverse)
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


print(conn.find_servos())
left = Arm("left", conn)

for hand in [left]:

    hand.start()

    pos = 1
    hand.thumb_rotation.move(pos)
    hand.thumb.move(pos)
    hand.index.move(pos)
    hand.middle.move(pos)
    hand.ring.move(pos)
    hand.little.move(pos)
    time.sleep(3)
    hand.stop()
