import argparse
from servos import get_servo
from servo import ServoConnection
import time

parser = argparse.ArgumentParser(description="Neutralize servo to 512")
parser.add_argument("port", type="str", nargs="?", help="Port")
parser.add_argument("servo", type=str, nargs="?", help="Servo (STS3215 or SCS0009)")
parser.add_argument("id", type=int, nargs="?", default=1, help="Current servo ID")
args = parser.parse_args()

servo = get_servo(args.servo)
conn = ServoConnection(args.port)

pos, _, _ = conn.read(args.id, servo.PRESENT_POSITION)
print(f"Current position {pos}")

conn.write(args.id, servo.TORQUE_ENABLE, 1)
conn.write(args.id, servo.GOAL_POSITION, servo.NEUTRAL_POS)
time.sleep(2)
conn.write(args.id, servo.TORQUE_ENABLE, 0)

pos, _, _ = conn.read(args.id, servo.PRESENT_POSITION)
print(f"Moved to {pos}")
