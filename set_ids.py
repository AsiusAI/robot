import argparse
import time
from servo import COMM_SUCCESS, ServoConnection, PortHandler
from servos import SCS0009

parser = argparse.ArgumentParser(description="Set servo IDs")
parser.add_argument("old_id", type=int, nargs="?", default=1, help="Current servo ID")
parser.add_argument("new_id", type=int, nargs="?", help="New servo ID")
args = parser.parse_args()

servo = SCS0009()
conn = ServoConnection("/dev/tty.usbmodem5A7A0572801", servo.BIG_ENDIAN)


def set_new_id(old_id, new_id):
    conn.write(old_id, servo.TORQUE_ENABLE, 0)
    conn.write(old_id, servo.LOCK, 0)
    conn.write(old_id, servo.ID, new_id)
    conn.write(new_id, servo.LOCK, 1)


found = conn.find_servos()
print(f"{found=}")

# Just setting one ID
if args.new_id:
    set_new_id(args.old_id, args.new_id)
    print(f"Set {args.old_id} to {args.new_id}")
    exit()

# Interactive
print(f"Waiting for ID {args.old_id}...")
while True:
    time.sleep(1)
    _, res, err = conn.ping(args.old_id)
    if res != COMM_SUCCESS:
        continue

    new_id = int(input("Found a new servo, what's the new ID?"))
    set_new_id(args.old_id, new_id)
    print(f"New servo ID set to {new_id}, looking for others...")
