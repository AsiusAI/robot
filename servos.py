from typing import Union


class STS3215:
    ENDIAN = False # True for big endian and False for small
    NEUTRAL_POS = 2048

    FIRMWARE_MAJOR_VERSION = (0, 1)
    FIRMWARE_MINOR_VERSION = (1, 1)
    MODEL_NUMBER = (3, 2)

    ID = (5, 1)
    BAUD_RATE = (6, 1)
    RETURN_DELAY_TIME = (7, 1)
    RESPONSE_STATUS_LEVEL = (8, 1)
    MIN_POSITION_LIMIT = (9, 2)
    MAX_POSITION_LIMIT = (11, 2)
    MAX_TEMPERATURE_LIMIT = (13, 1)
    MAX_VOLTAGE_LIMIT = (14, 1)
    MIN_VOLTAGE_LIMIT = (15, 1)
    MAX_TORQUE_LIMIT = (16, 2)
    PHASE = (18, 1)
    UNLOADING_CONDITION = (19, 1)
    LED_ALARM_CONDITION = (20, 1)
    P_COEFFICIENT = (21, 1)
    D_COEFFICIENT = (22, 1)
    I_COEFFICIENT = (23, 1)
    MINIMUM_STARTUP_FORCE = (24, 2)
    CW_DEAD_ZONE = (26, 1)
    CCW_DEAD_ZONE = (27, 1)
    PROTECTION_CURRENT = (28, 2)
    ANGULAR_RESOLUTION = (30, 1)
    HOMING_OFFSET = (31, 2)
    OPERATING_MODE = (33, 1)
    PROTECTIVE_TORQUE = (34, 1)
    PROTECTION_TIME = (35, 1)
    OVERLOAD_TORQUE = (36, 1)
    VELOCITY_CLOSED_LOOP_P_PROPORTIONAL_COEFFICIENT = (37, 1)
    OVER_CURRENT_PROTECTION_TIME = (38, 1)
    VELOCITY_CLOSED_LOOP_I_INTEGRAL_COEFFICIENT = (39, 1)

    # SRAM
    TORQUE_ENABLE = (40, 1)
    ACCELERATION = (41, 1)
    GOAL_POSITION = (42, 2)
    GOAL_TIME = (44, 2)
    GOAL_VELOCITY = (46, 2)
    TORQUE_LIMIT = (48, 2)
    LOCK = (55, 1)
    PRESENT_POSITION = (56, 2)  # read-only
    PRESENT_VELOCITY = (58, 2)  # read-only
    PRESENT_LOAD = (60, 2)  # read-only
    PRESENT_VOLTAGE = (62, 1)  # read-only
    PRESENT_TEMPERATURE = (63, 1)  # read-only
    STATUS = (65, 1)  # read-only
    MOVING = (66, 1)  # read-only
    PRESENT_CURRENT = (69, 2)  # read-only
    GOAL_POSITION_2 = (71, 2)  # read-only

    # Factory
    MOVING_VELOCITY = (80, 1)
    MOVING_VELOCITY_THRESHOLD = (80, 1)
    DTS = (81, 1)  # (ms)
    VELOCITY_UNIT_FACTOR = (82, 1)
    HTS = (83, 1)  # (ns), valid for firmware >= 2.54, other versions keep 0
    MAXIMUM_VELOCITY_LIMIT = (84, 1)
    MAXIMUM_ACCELERATION = (85, 1)
    ACCELERATION_MULTIPLIER = (86, 1)  # in effect when acceleration is 0

    BAUDRATE_TABLE = {
        1_000_000: 0,
        500_000: 1,
        250_000: 2,
        128_000: 3,
        115_200: 4,
        57_600: 5,
        38_400: 6,
        19_200: 7,
    }

    RESOLUTION = 4096
    SERIES_ENCODINGS_TABLE = {
        "Homing_Offset": 11,
        "Goal_Velocity": 15,
        "Present_Velocity": 15,
    }
    MODEL_NUMBER = 777
    PROTOCOL = 0


# http://doc.feetech.cn/#/prodinfodownload?srcType=FT-SCSCL-emanual-cbcc8ab2e3384282a01d4bf3
class SCS0009:
    ENDIAN = True
    NEUTRAL_POS = 512
    # EPROM
    FIRMWARE_MAJOR_VERSION = (0, 1)
    FIRMWARE_MINOR_VERSION = (1, 1)
    MODEL_NUMBER = (3, 2)

    ID = (5, 1)
    BAUD_RATE = (6, 1)
    RETURN_DELAY_TIME = (7, 1)
    RESPONSE_STATUS_LEVEL = (8, 1)
    MIN_POSITION_LIMIT = (9, 2)
    MAX_POSITION_LIMIT = (11, 2)
    MAX_TEMPERATURE_LIMIT = (13, 1)
    MAX_VOLTAGE_LIMIT = (14, 1)
    MIN_VOLTAGE_LIMIT = (15, 1)
    MAX_TORQUE_LIMIT = (16, 2)
    PHASE = (18, 1)
    UNLOADING_CONDITION = (19, 1)
    LED_ALARM_CONDITION = (20, 1)
    P_COEFFICIENT = (21, 1)
    D_COEFFICIENT = (22, 1)
    I_COEFFICIENT = (23, 1)
    MINIMUM_STARTUP_FORCE = (24, 2)
    CW_DEAD_ZONE = (26, 1)
    CCW_DEAD_ZONE = (27, 1)
    PROTECTIVE_TORQUE = (37, 1)
    PROTECTION_TIME = (38, 1)

    # SRAM
    TORQUE_ENABLE = (40, 1)
    ACCELERATION = (41, 1)
    GOAL_POSITION = (42, 2)
    RUNNING_TIME = (44, 2)
    GOAL_VELOCITY = (46, 2)
    LOCK = (48, 1)
    PRESENT_POSITION = (56, 2)  # read-only
    PRESENT_VELOCITY = (58, 2)  # read-only
    PRESENT_LOAD = (60, 2)  # read-only
    PRESENT_VOLTAGE = (62, 1)  # read-only
    PRESENT_TEMPERATURE = (63, 1)  # read-only
    SYNC_WRITE_FLAG = (64, 1)  # read-only
    STATUS = (65, 1)  # read-only
    MOVING = (66, 1)  # read-only

    # Factory
    PWM_MAXIMUM_STEP = (78, 1)
    MOVING_VELOCITY_THRESHOLD_X50 = (79, 1)
    DTS = (80, 1)  # (ms)
    MINIMUM_VELOCITY_LIMIT_X50 = (81, 1)
    MAXIMUM_VELOCITY_LIMIT_X50 = (82, 1)
    ACCELERATION_2 = (83, 1)  # unclear meaning

    BAUDRATE_TABLE = {
        1_000_000: 0,
        500_000: 1,
        250_000: 2,
        128_000: 3,
        115_200: 4,
        57_600: 5,
        38_400: 6,
        19_200: 7,
    }

    RESOLUTION = 1024
    MODEL_NUMBER = 1284
    PROTOCOL = 1


def get_servo(type: str):
    return STS3215 if type == "STS3215" else SCS0009
