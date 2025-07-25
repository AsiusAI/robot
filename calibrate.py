import argparse
from servos import get_servo
from servo import Port, ServoConnection
import time

parser = argparse.ArgumentParser(description="Calibrate")
parser.add_argument("port", type=str, nargs="?", help="Port")
parser.add_argument("servo", type=str, nargs="?", help="Servo (STS3215 or SCS0009)")
args = parser.parse_args()

port = Port(args.port)
conn = get_servo(args.servo, port)

print(conn.find_servos())

while True:
    id = int(input("Servo ID: "))

    curr_min, _, _ = conn.read(id, conn.MIN_POSITION_LIMIT)
    curr_max, _, _ = conn.read(id, conn.MAX_POSITION_LIMIT)
    pos, _, _ = conn.read(id, conn.PRESENT_POSITION)
    print(f"Calibrating servo with {id=} {curr_min=} {curr_max=} {pos=}")

    print("Capturing the servo movements for 10s...")
    positions = []
    for i in range(100):
        pos, _, _ = conn.read(id, conn.PRESENT_POSITION)
        positions.append(pos)
        time.sleep(0.1)
    max(positions)

    print(f"Setting the min to {min(positions)} and max to {max(positions)}")
    conn.write(id, conn.LOCK, 0)
    conn.write(id, conn.MIN_POSITION_LIMIT, min(positions))
    conn.write(id, conn.MAX_POSITION_LIMIT, max(positions))
    conn.write(id, conn.LOCK, 1)
    print(f"Done!")
