import argparse
from servos import STS3215, SCS0009
from servo import PortHandler, ServoConnection
import time

parser = argparse.ArgumentParser(description="Neutralize servo to 512")
parser.add_argument("id", type=int, nargs="?", default=1, help="Current servo ID")
args = parser.parse_args()

servo = SCS0009()
conn = ServoConnection("/dev/tty.usbmodem5A7A0572801", servo.BIG_ENDIAN)

pos, _, _ = conn.read(args.id, servo.PRESENT_POSITION)
print(f"Current position {pos}")

conn.write(args.id, servo.TORQUE_ENABLE, 1)
conn.write(args.id, servo.GOAL_POSITION, 512)
time.sleep(2)
conn.write(args.id, servo.TORQUE_ENABLE, 0)

pos, _, _ = conn.read(args.id, servo.PRESENT_POSITION)
print(f"Moved to {pos}")
