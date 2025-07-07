import threading
import odrive
import time
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL

# Global variables for cleanup
odrv0 = None
axis0 = None
axis1 = None

# Global variables for status
battery_voltage = 0.0
motor_currents = [0.0, 0.0]
uptime = 0


def get_battery_status():
    """Get battery and motor status from ODrive"""
    global battery_voltage, motor_currents, axis0, axis1, odrv0
    try:
        if odrv0 and axis0 and axis1:
            battery_voltage = odrv0.vbus_voltage
            motor_currents = [
                axis0.motor.current_control.Iq_measured,
                axis1.motor.current_control.Iq_measured,
            ]
    except Exception as e:
        print(f"Error reading status: {e}")


def status_monitor():
    """Background thread to monitor system status"""
    global uptime
    start_time = time.time()
    while True:
        uptime = int(time.time() - start_time)
        get_battery_status()
        time.sleep(1)


def cleanup_motors():
    """Cleanup function to stop motors and return to idle state"""
    global axis0, axis1
    if axis0 and axis1:
        print("Returning to idle state...")
        try:
            axis0.requested_state = AXIS_STATE_IDLE
            axis1.requested_state = AXIS_STATE_IDLE
        except Exception as e:
            print(f"Error during cleanup: {e}")


def start_odrive():
    global axis0, axis1, odrv0
    try:
        odrv0 = odrive.find_any(timeout=3)
        axis0 = odrv0.axis0
        axis1 = odrv0.axis1

        print("\nStarting motors")
        axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    except:
        print("Odrive failed")


def move(left, right, speed):
    global axis0, axis1
    if axis0 and axis1:
        axis0.controller.input_vel = right * speed
        axis1.controller.input_vel = left * -1 * speed

status_thread = threading.Thread(target=status_monitor, daemon=True)
status_thread.start()

def get_status():
    global battery_voltage, motor_currents, uptime
    print(battery_voltage,motor_currents,uptime)
    motor_current = (
        max(abs(current) for current in motor_currents) if motor_currents else 0
    )

    return {
        "battery_voltage": battery_voltage,
        "motor_current": motor_current,
        "uptime": uptime,
    }