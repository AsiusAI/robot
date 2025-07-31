import argparse
import os
from servos import get_servo
from servo import ServoConnection,Port
import time

parser = argparse.ArgumentParser(description='Neutralize servo to 512')
parser.add_argument('port', type=str, nargs='?', default=os.getenv("PORT"), help='Port')
parser.add_argument('id', type=int, nargs='?', default=1, help='Current servo ID')
args = parser.parse_args()

port = Port(args.port)
servo = get_servo("SCS0009", port)

pos, _, _ = servo.read(args.id, servo.PRESENT_POSITION)
print(f'Current position {pos}')

servo.write(args.id, servo.TORQUE_ENABLE, 1)
servo.write(args.id, servo.GOAL_POSITION, servo.NEUTRAL_POS)
time.sleep(2)
servo.write(args.id, servo.TORQUE_ENABLE, 0)

pos, _, _ = servo.read(args.id, servo.PRESENT_POSITION)
print(f'Moved to {pos}')
