import threading
import odrive
import time
from odrive.enums import AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL

from robots import Status

# Global variables for cleanup
odrv0 = None
axis0 = None
axis1 = None

start_time = time.time()


def get_status() -> Status:
  uptime = int(time.time() - start_time)

  if not odrv0 or not axis0 or not axis1:
    return Status(0.0, 0.0, uptime)

  voltage = odrv0.vbus_voltage
  currents = [
    axis0.motor.current_control.Iq_measured,
    axis1.motor.current_control.Iq_measured,
  ]
  current = max(abs(current) for current in currents) if currents else 0
  return Status(voltage, current, uptime)


def cleanup_motors():
  """Cleanup function to stop motors and return to idle state"""
  global axis0, axis1
  if axis0 and axis1:
    print('Returning to idle state...')
    try:
      axis0.requested_state = AXIS_STATE_IDLE
      axis1.requested_state = AXIS_STATE_IDLE
    except Exception as e:
      print(f'Error during cleanup: {e}')


def start_odrive():
  global axis0, axis1, odrv0
  try:
    odrv0 = odrive.find_any(timeout=3)
    axis0 = odrv0.axis0
    axis1 = odrv0.axis1

    print('\nStarting motors')
    axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
  except:
    print('Odrive failed')


def move(left, right, speed):
  global axis0, axis1
  if axis0 and axis1:
    axis0.controller.input_vel = right * speed
    axis1.controller.input_vel = left * -1 * speed


def control(x, y, speed):
  left = y + x
  right = y - x

  left = max(-1, min(1, left))
  right = max(-1, min(1, right))

  move(left=left, right=right, speed=speed)
