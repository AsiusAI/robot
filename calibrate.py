from servos import STS3215, SCS0009
from servo import PortHandler, Connection
import time

servo = SCS0009()
port = PortHandler("/dev/tty.usbmodem5A7A0572801")
conn = Connection(servo.BIG_ENDIAN, port)

SERVOS = [
    (31, 409, 588, True),
    (32, 372, 786, True),
    (33, 6, 210, True),
    (34, 610, 815, False),
    (35, 85, 295, True),
    (36, 615, 818, False),
]
try:
    conn.port.ser.reset_input_buffer()
    for id, min, max, _ in SERVOS:
        pos, _, _ = conn.read(id, servo.PRESENT_POSITION)
        curr_min, _, _ = conn.read(id, servo.MIN_POSITION_LIMIT)
        curr_max, _, _ = conn.read(id, servo.MAX_POSITION_LIMIT)
        print(f"{id=} {curr_min=} {curr_max=} {pos=}")

        # Set new min/max
        conn.write(id, servo.LOCK, 0)
        conn.write(id, servo.MIN_POSITION_LIMIT, min)
        conn.write(id, servo.MAX_POSITION_LIMIT, max)
        conn.write(id, servo.LOCK, 1)

    # Start
    for id, min, max, reversed in SERVOS:
        conn.write(id, servo.TORQUE_ENABLE, 1)

    for _ in range(3):
      # Open hand
      for id, min, max, reversed in SERVOS:
          conn.write(id, servo.GOAL_POSITION, max if reversed else min)
      time.sleep(2)

      # Close hand
      for id, min, max, reversed in SERVOS:
          conn.write(id, servo.GOAL_POSITION, min if reversed else max)
      time.sleep(2)

      # Middle
      for id, min, max, reversed in SERVOS:
          conn.write(
              id,
              servo.GOAL_POSITION,
              int(max + ((min - max) / 2)) if reversed else int(min + ((max - min) / 2)),
          )
      time.sleep(2)

    # Stop
    for id, _, _, _ in SERVOS:
        conn.write(id, servo.TORQUE_ENABLE, 0)

finally:
    port.closePort()
