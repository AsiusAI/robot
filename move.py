import argparse
import os
import time
from arms import Arm, Hand
from servo import Port

parser = argparse.ArgumentParser(description='Move')
parser.add_argument('--port', type=str, default=os.getenv('PORT'), help='Port')
args = parser.parse_args()

port = Port(args.port)

right = Hand('right', port)
servos = right.servos


for servo in servos:
  servo.start()

for _ in range(3):
  for servo in servos:
    servo.move(1)

  time.sleep(2)
  
  for servo in servos:servo.move(0)
  time.sleep(2)

for servo in servos:
  servo.stop()