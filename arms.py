from servo import COMM_SUCCESS, ServoConnection
from servos import SCS0009, STS3215


class Servo:
  def __init__(self, id, name, conn: STS3215 | SCS0009, reverse: bool):
    self.id = id
    self.name = name
    self.conn = conn
    self.reverse = reverse

    _, res, _ = conn.ping(id)
    if res != COMM_SUCCESS:
      raise Exception(f'{name} ({id}) not connected')

    self.min, _, _ = conn.read(id, conn.MIN_POSITION_LIMIT)
    self.max, _, _ = conn.read(id, conn.MAX_POSITION_LIMIT)

  def move(self, pos: float):
    relative = abs(self.min - self.max) * pos
    pos = (self.max - relative) if self.reverse else (self.min + relative)
    self.conn.write(self.id, self.conn.GOAL_POSITION, int(pos))

  def pos(self):
    data, res, err = self.conn.read(self.id, self.conn.PRESENT_POSITION)
    if res != COMM_SUCCESS:
      print(f'Getting pos failed for {self.name} ({self.id}): {res}')
      return None
    return data

  def start(self):
    self.conn.write(self.id, self.conn.TORQUE_ENABLE, 1)

  def stop(self):
    self.conn.write(self.id, self.conn.TORQUE_ENABLE, 0)

  def __repr__(self):
    return f"Servo(id={self.id}, name='{self.name}', type={self.conn.__class__.__name__}, range=({self.min} → {self.max}), reverse={self.reverse})"


class Hand:
  def __init__(self, side, port: ServoConnection):
    self.side = side
    start = 30 if self.side == 'right' else 40

    reverse = side == 'right'
    scs = SCS0009(port)
    self.thumb_rotation = Servo(start + 1, 'thumb_rotation', scs, reverse)
    self.thumb = Servo(start + 2, 'thumb', scs, reverse)
    self.index = Servo(start + 3, 'index', scs, reverse)
    self.middle = Servo(start + 4, 'middle', scs, not reverse)
    self.ring = Servo(start + 5, 'ring', scs, reverse)
    self.little = Servo(start + 6, 'little', scs, not reverse)
    self.servos = [
      self.thumb_rotation,
      self.thumb,
      self.index,
      self.middle,
      self.ring,
      self.little,
    ]

  # def start(self):
  #     for servo in self.servos:
  #         servo.start()

  # def stop(self):
  #     for servo in self.servos:
  #         servo.stop()


class Arm:
  def __init__(self, side, port):
    sts = STS3215(port)
    start = 10 if side == 'right' else 20
    reverse = side == 'right'
    self.shoulder_rot = Servo(start + 1, 'shoulder_rot', sts, reverse)
    self.shoulder = Servo(start + 2, 'shoulder', sts, reverse)
    self.upperarm_rot = Servo(start + 3, 'upperarm_rot', sts, reverse)
    self.elbow = Servo(start + 4, 'elbow', sts, reverse)
    self.forearm_rot = Servo(start + 5, 'forearm_rot', sts, reverse)
    self.hand_yaw = Servo(start + 6, 'hand_yaw', sts, reverse)
    self.hand_pitch = Servo(start + 7, 'hand_pitch', sts, reverse)
    self.servos = [
      self.shoulder_rot,
      self.shoulder,
      self.upperarm_rot,
      self.elbow,
      self.forearm_rot,
      self.hand_yaw,
      self.hand_pitch,
    ]
