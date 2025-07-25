    JOINTS = [21, 22, 23, 24, 25, 26, 27]
    def move(id, pos: int, sleep=3):
        print(f"moving to {pos}")
        conn.write(id, servo.GOAL_POSITION, pos)
        time.sleep(sleep)
        current_pos = get_current_pos(id)
        print(f"{current_pos=}")
    def start(id):
        conn.write(id, servo.TORQUE_ENABLE, 1)
        conn.write(id, servo.ACCELERATION, 0xFF)
        conn.write(id, servo.GOAL_VELOCITY, 0xFFFF)

    def stop(id):
        conn.write(id, servo.TORQUE_ENABLE, 0)
