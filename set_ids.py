import argparse
import os
import time
from servo import COMM_SUCCESS, Port
from servos import get_servo

parser = argparse.ArgumentParser(description='Set servo IDs')
parser.add_argument('--port', type=str, default=os.getenv("PORT"), help='Port')
parser.add_argument('--servo', type=str, help='Servo (SCS0009 or STS3215)')
parser.add_argument('--id', type=int, default=1, help='Current servo ID')
parser.add_argument('--new-id', type=int, help='New servo ID')
args = parser.parse_args()

port = Port(args.port)
conn = get_servo(args.servo, port)


def set_new_id(id, new_id):
  conn.write(id, conn.TORQUE_ENABLE, 0)
  conn.write(id, conn.LOCK, 0)
  conn.write(id, conn.ID, new_id)
  conn.write(new_id, conn.LOCK, 1)


found = conn.find_servos()
print(f'{found=}')

# Just setting one ID
if args.new_id:
  set_new_id(args.id, args.new_id)
  print(f'Set {args.id} to {args.new_id}')
  exit()

# Interactive
print(f'Waiting for ID {args.id}...')
while True:
  time.sleep(1)
  _, res, err = conn.ping(args.id)
  if res != COMM_SUCCESS:
    continue

  new_id = int(input("Found a new servo, what's the new ID?"))
  set_new_id(args.id, new_id)
  print(f'New servo ID set to {new_id}, looking for others...')
