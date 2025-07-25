
servo = SCS0009()
port = PortHandler("/dev/tty.usbmodem5A7A0572801")
conn = Connection(0, port)

if not port.openPort():
    print("Failed to open the port")
    exit()

if not port.setBaudRate(1_000_000):
    print("Setting baud rate failed")
    exit()



def find_servos():
    found = []
    for id in range(0, 0xFE):
        _, res, err = conn.ping(id)
        if res == COMM_SUCCESS:
            print(id)
            found.append(id)
    return found


def set_new_id(old_id, new_id):
    print(conn.write(old_id, servo.TORQUE_ENABLE, 0))
    print(conn.write(old_id, servo.LOCK, 0))
    print(conn.write(old_id, servo.ID, new_id))
    print(conn.write(new_id, servo.LOCK, 1))
