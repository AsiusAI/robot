import argparse
import math
import os
import time
import threading
from arms import Arm, Hand
from servo import Port
from servos import STS3215

parser = argparse.ArgumentParser(description='Calibrate')
parser.add_argument('port', type=str, nargs='?', default=os.getenv('PORT'), help='Port')
parser.add_argument('--side', type=str, help='left/right')
parser.add_argument('--part', type=str, help='arm/hand')
args = parser.parse_args()
if not args.port:
  raise Exception('Port is required!')
if not args.side or not args.part:
  raise Exception('Side and part are required')

port = Port(args.port)
part = Arm(args.side, port) if args.part == 'arm' else Hand(args.side, port)
servos = part.servos


# Enabling multi-turn position for STS3215
if args.part == 'arm':
  for servo in servos:
    phase, _, _ = servo.conn.read(servo.id, servo.conn.PHASE)
    if phase != 28:
      servo.conn.write(servo.id, servo.conn.LOCK, 0)
      print(servo.conn.write(servo.id, servo.conn.OPERATING_MODE, 0))
      print(servo.conn.write(servo.id, servo.conn.PHASE, 28))  # The stock value was 12, so added 16 to flip 4th bit
      servo.conn.write(servo.id, servo.conn.LOCK, 1)


# Stop on enter
stop_flag = False


def wait_for_enter():
  input('Press ENTER to stop...\n')
  global stop_flag
  stop_flag = True


threading.Thread(target=wait_for_enter, daemon=True).start()


def print_table(data):
  lines = []
  lines.append(f'{"ID":<6}{"Name":<20}{"Current":<10}{"Min (Old min)":<20}{"Max (Old max)":<20}')
  lines.append('-' * 76)
  for sid, (name, pos, minv, old_min, maxv, old_max) in sorted(data.items()):
    lines.append(f'{sid:<6}{name:<20}{f"{pos}":<10}{f"{minv} ({old_min})":<20}{f"{maxv} ({old_max})":<20}')
  table = '\n'.join(lines)
  print(f'\033[{len(lines)}F', end='')
  print(table)


# Calibration
# (name, pos, min, old_min, max, old_max)
positions = {}
while not stop_flag:
  for servo in servos:
    _, _, prev_min, _, prev_max, _ = positions.get(servo.id, (servo.name, 0, math.inf, servo.min, 0, servo.max))
    pos = servo.pos()
    # If the value is too small, then we zero the position again
    if args.part == 'arm' and pos < 50:
      servo.conn.write(servo.id, servo.conn.TORQUE_ENABLE, 128)
      pos = servo.pos()
      positions[servo.id] = (servo.name, pos, math.inf, servo.min, 0, servo.max)
      continue
    positions[servo.id] = (
      servo.name,
      pos,
      min(pos, prev_min) if pos is not None else prev_min,
      servo.min,
      max(pos, prev_max) if pos is not None else prev_max,
      servo.max,
    )

  print_table(positions)
  time.sleep(0.02)

# Saving min and max
for servo in servos:
  _, _, min, _, max, _ = positions.get(servo.id)
  servo.conn.write(servo.id, servo.conn.LOCK, 0)
  servo.conn.write(servo.id, servo.conn.MIN_POSITION_LIMIT, min)
  servo.conn.write(servo.id, servo.conn.MAX_POSITION_LIMIT, max)
  servo.conn.write(servo.id, servo.conn.LOCK, 1)

print('Calibration finished.')

# Setting straight position

if args.part == 'arm':
  input('Set all servos to straight position, then press enter: ')
  for servo in servos:
    if isinstance(servo.conn, STS3215):
      pos = servo.pos()
      servo.conn.write(servo.id, servo.conn.LOCK, 0)
      servo.conn.write(servo.id, servo.conn.HOMING_OFFSET, pos)
      servo.conn.write(servo.id, servo.conn.LOCK, 1)
