import argparse
import os
import time
from arms import Arm
from servo import Port

parser = argparse.ArgumentParser(description="Move")
parser.add_argument("port", type=str, nargs="?", default=os.getenv("PORT"), help="Port")
args = parser.parse_args()

port = Port(args.port)

right = Arm("right", port)
left = Arm("left", port)

for hand in [right,left]:
    hand.start()

    pos = 0
    hand.thumb_rotation.move(pos)
    hand.thumb.move(pos)
    hand.index.move(pos)
    hand.middle.move(pos)
    hand.ring.move(pos)
    hand.little.move(pos)
    time.sleep(5)
    hand.stop()
