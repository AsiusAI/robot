from servos import STS3215, SCS0009
from servo import PortHandler, Connection
import time

servo = SCS0009()
port = PortHandler("/dev/tty.usbmodem5A7A0572801")
conn = Connection(0, port)

if not port.openPort():
    print("Failed to open the port")
    exit()

if not port.setBaudRate(1_000_000):
    print("Setting baud rate failed")
    exit()



id = 1
try:
    conn.port.ser.reset_input_buffer()

    print(conn.read(id,servo.PRESENT_POSITION))
    print(conn.read(id,servo.GOAL_POSITION))
    print(conn.read(id,servo.MIN_POSITION_LIMIT))
    print(conn.read(id,servo.MAX_POSITION_LIMIT))
    print(conn.read(id,servo.BAUD_RATE))

    print(conn.write(id,servo.LOCK,0))
    print(conn.write(id,servo.MIN_POSITION_LIMIT, 20))
    print(conn.write(id,servo.MAX_POSITION_LIMIT,1003))
    print(conn.write(id,servo.LOCK,1))


    print(conn.write(id, servo.TORQUE_ENABLE, 1))
    print(conn.write(id,servo.GOAL_POSITION, 200))
    time.sleep(1)

    print(conn.read(id,servo.GOAL_POSITION))
    print(conn.write(id, servo.TORQUE_ENABLE, 0))

finally:
    port.closePort()
