from servos import STS3215, SCS0009
from servo import PortHandler, ServoConnection
import time

servo = SCS0009()
conn = ServoConnection("/dev/tty.usbmodem5A7A0572801",servo.BIG_ENDIAN)

id = 1
pos, _, _ = conn.read(id, servo.PRESENT_POSITION)
curr_min, _, _ = conn.read(id, servo.MIN_POSITION_LIMIT)
curr_max, _, _ = conn.read(id, servo.MAX_POSITION_LIMIT)
print(f"{curr_min=} {curr_max=} {pos=}")

conn.write(id, servo.TORQUE_ENABLE, 1)

conn.write(id, servo.GOAL_POSITION, 512)
time.sleep(2)

conn.write(id, servo.TORQUE_ENABLE, 0)
pos, _, _ = conn.read(id, servo.PRESENT_POSITION)
print(f"{pos=}")

