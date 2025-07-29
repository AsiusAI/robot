import argparse
import os
from arms import Arm
from servo import Port

parser = argparse.ArgumentParser(description='Move')
parser.add_argument('port', type=str, nargs='?', default=os.getenv('PORT'), help='Port')
args = parser.parse_args()

port = Port(args.port)

right = Arm('right', port)

# servo = right.hand_pitch

# servo.conn.write(servo.id, servo.conn.LOCK, 0)
# print(servo.conn.write(servo.id, servo.conn.OPERATING_MODE, 0))
# print(servo.conn.write(servo.id, servo.conn.PHASE, 28))
# print(servo.conn.write(servo.id, servo.conn.MIN_POSITION_LIMIT, 0))
# print(servo.conn.write(servo.id, servo.conn.MAX_POSITION_LIMIT, 25000))
# print(servo.conn.write(servo.id, servo.conn.TORQUE_ENABLE, 128))
# servo.conn.write(servo.id, servo.conn.LOCK, 1)
# # print(servo)

# servo.start()
# print(servo.conn.read(servo.id, servo.conn.PRESENT_POSITION))
# print(servo.conn.write(servo.id, servo.conn.GOAL_POSITION, 32767))
# time.sleep(4)
# servo.stop()

for servo in right.servos:
  print(servo.conn.read(servo.id, servo.conn.HOMING_OFFSET))
