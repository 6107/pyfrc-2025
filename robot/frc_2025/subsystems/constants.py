# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #
#
# Constants for source in this subdirectory will go here

from enum import IntEnum, unique

from phoenix6.controls.position_voltage import PositionVoltage


###############################################################################
# 2025 Device IDS

@unique
class DeviceID(IntEnum):
    # Other drivetrain constants are available in the swervedrive subdirectory
    DRIVETRAIN_LEFT_FRONT_TURNING_ID = 21
    DRIVETRAIN_LEFT_FRONT_DRIVING_ID = 22
    DRIVETRAIN_LEFT_FRONT_ENCODER_ID = 31

    DRIVETRAIN_RIGHT_FRONT_TURNING_ID = 23
    DRIVETRAIN_RIGHT_FRONT_DRIVING_ID = 24
    DRIVETRAIN_RIGHT_FRONT_ENCODER_ID = 32

    DRIVETRAIN_LEFT_REAR_TURNING_ID = 25
    DRIVETRAIN_LEFT_REAR_DRIVING_ID = 26
    DRIVETRAIN_LEFT_REAR_ENCODER_ID = 33

    DRIVETRAIN_RIGHT_REAR_TURNING_ID = 27
    DRIVETRAIN_RIGHT_REAR_DRIVING_ID = 28
    DRIVETRAIN_RIGHT_REAR_ENCODER_ID = 34

    # Elevator
    ELEVATOR_DEVICE_ID = 10

    # Alge Subsystem
    ALGE_ROLLER_DEVICE_ID = 11
    ALGE_ROTATION_DEVICE_ID = 12

    # Coral Subsystem
    INTAKE_LEFT_DEVICE_ID = 13
    INTAKE_RIGHT_DEVICE_ID = 14
    INTAKE_EXTEND_DEVICE_ID = 15

#########################
# Intake constants

D_SHOOT_SPEED = 0.5
D_ALGE_HOLD_SPEED = -0.05
D_CORAL_INTAKE_SPEED = 0.1
D_ALGE_INTAKE_SPEED = -0.25

D_ALGE_GRABBER_GRAB = -0.25
D_ALGE_GRABBER_HOLD = -0.05
D_ALGE_GRABBER_SHOOT = 0.25

I_ALGE_ROTATION_IN = 0
I_ALGE_ROTATION_OUT = 1

#########################
# Elevator constants

EL_POS_L0 = PositionVoltage(0.0, slot=0)
EL_POS_L1 = PositionVoltage(0.0, slot=0)
EL_POS_L2 = PositionVoltage(0.0, slot=0)
EL_POS_L3 = PositionVoltage(0.0, slot=0)
EL_POS_IN = PositionVoltage(0.0, slot=0)

# starting position for odometry
k_start_x = 0
k_start_y = 0

k_has_front_camera = False
k_use_vision_odometry = False

k_swerve_debugging_messages = True
# multiple attempts at tags this year - TODO - use l/r/ or up/down tilted cameras again, gives better data
k_use_apriltag_odometry = True
k_use_quest_odometry = False
k_use_photontags = False  # take tags from photonvision camera
k_use_CJH_tags = True  # take tags from the pis
k_swerve_only = False
k_swerve_rate_limited = True
k_field_oriented = True
