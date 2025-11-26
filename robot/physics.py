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
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import logging

from wpilib import Encoder, Field2d, SmartDashboard
from wpilib.simulation import EncoderSim, DCMotorSim, DifferentialDrivetrainSim
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveKinematics
from wpimath.system.plant import DCMotor
from wpimath.units import metersToFeet

import hal.simulation

from pyfrc.physics import drivetrains
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics.units import units

from robot import MyRobot, RobotContainer
from frc_2026.subsystems.xrp_differential_drive import XrpDifferentialDriveSubsystem
from lib_6107.constants import XrpConstants

logger = logging.getLogger(__name__)


class PhysicsEngine:
    """
    Simulates a 2-wheel XRP robot using Arcade Drive joystick control.

    Any objects created or manipulated in this file are for simulation purposes only.
    """
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        logger.info("PhysicsEngine: entry")

        self._physics_controller = physics_controller
        self._robot: MyRobot = robot
        self._container: RobotContainer = robot.container
        # self._drive: XrpDifferentialDriveSubsystem = robot.container.drive
        # self._controller = robot.container.controller
        #
        # # NOTE: Depending on the library, the units may need to either be standard python types,
        # #       but in a few, they require Quantity objects created by the 'pint' moduled. It is
        # #       really a bit ad-hoc and some of the libraries want wpimath.units.* objectss, but'
        # #       those are alias for the python types and the libraries do not work.
        # def xrp_dc_motor() -> DCMotor:
        #     return DCMotor(XrpConstants.Motor.Voltage,      # Nominal voltage - Voltage at which the motor constants were measured.
        #                    XrpConstants.Motor.StallTorque,  # Stall torque in N*m (approximation) - Torque when stalled.
        #                    XrpConstants.Motor.StallCurrent, # Stall current - Current draw when stalled.
        #                    XrpConstants.Motor.FreeCurrent,  # Free Current - Current draw under no load.
        #                    XrpConstants.Motor.FreeSpeed,    # Angular velocity under no load.
        #                    XrpConstants.Motor.MotorCount)   # Number of motors in a gearbox.
        #
        # # Initialize a two-motor drivetrain model
        # wheelbase = metersToFeet(XrpConstants.Physical.TrackWidth) * units.feet
        # speed = (XrpConstants.Motor.FreeSpeed * units.meters / units.seconds).to(units.fps)
        #
        # self._drivetrain = drivetrains.TwoMotorDrivetrain(x_wheelbase=wheelbase,
        #                                                   speed=speed,
        #                                                   deadzone=drivetrains.linear_deadzone(0.1))
        # # Create simulated motors
        # self._left_motor: DCMotor = xrp_dc_motor()
        # self._right_motor: DCMotor = xrp_dc_motor()
        #
        # # Create encoder simulators (12 CPR on motor output shaft, so 585 counts per wheel revolution)
        # # TODO: Investigate -> Encoders are accessed via SimDeviceSim in RobotPy for XRP
        # self._left_encoder_sim = EncoderSim(Encoder(XrpConstants.EncoderChannel.LeftChannel_A,
        #                                             XrpConstants.EncoderChannel.LeftChannel_B))
        # # self._left_encoder_distance = self._left_encoder_sim.getDistance()
        # # self._left_encoder_rate = self._left_encoder_sim.getRate()
        #
        # self._right_encoder_sim = EncoderSim(Encoder(XrpConstants.EncoderChannel.RightChannel_A,
        #                                             XrpConstants.EncoderChannel.RightChannel_B))
        # # self._right_encoder_distance = self._right_encoder_sim.getDistance()
        # # self._right_encoder_rate = self._right_encoder_sim.getRate()
        # #
        # # # Track motor inputs (which are set in the main robot code)
        # # self._left_motor_input = 0.0
        # # self._right_motor_input = 0.0

        self._physics_controller.field.setRobotPose(Pose2d(0.5, 2.0, Rotation2d(0)))

        # Set up field
        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)


    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # controller_forward_speed = self._controller.getY()
        # controller_rotation_speed = self._controller.getX()
        #
        # log_it =  controller_forward_speed or controller_rotation_speed
        # if log_it:
        #     logger.info(f"motor: left: {controller_forward_speed}, right: {controller_rotation_speed}")
        #
        # left_encoder_distance = self._left_encoder_sim.getDistance()
        # left_encoder_rate = self._left_encoder_sim.getRate()
        #
        # right_encoder_distance = self._right_encoder_sim.getDistance()
        # right_encoder_rate = self._right_encoder_sim.getRate()
        #
        # # Gyro TODO: Future
        # #self.gyro = wpilib.simulation.AnalogGyroSim(robot.gyro)
        # if log_it:
        #     logger.info(f"encoder: left: {left_encoder_distance}/{left_encoder_rate}, "
        #                 f"right: {right_encoder_distance}/{right_encoder_rate}")
        # # Pressing the 'w' up, results in the left & right rate of change increasing a small amount
        # # that does not increase like speed. Change is .00556, but the distance for left
        # # and right stays at zero.
        #
        # left, right = self._drive.left_motor.get(), self._drive.right_motor.get()
        # if left or right:
        #     logger.info(f"XPRMotor: Speed, Left: {left}, right: {right}")
        #
        # #chassis_speed: ChassisSpeeds = self._drivetrain.calculate(controller_forward_speed, controller_rotation_speed)
        # chassis_speed: ChassisSpeeds = self._drivetrain.calculate(left, right)
        #
        # wheel_speeds = DifferentialDriveKinematics(XrpConstants.Physical.TrackWidth).toWheelSpeeds(chassis_speed)
        # if log_it:
        #     logger.info(f"chassis_speed: {chassis_speed.vx}/{chassis_speed.vy}/{chassis_speed.omega}")
        #     logger.info(f"chassis_speed: {chassis_speed.vx_fps}/{chassis_speed.vy_fps}/{chassis_speed.omega}")
        #     logger.info(f"wheel_speeds: {wheel_speeds}")
        # # Pressing the 'w' up, and holding does not change wheel or chassis speed
        #
        # self._physics_controller.drive(chassis_speed, tm_diff)
        #
        # # optional: compute encoder
        # l_encoder = self._drivetrain.wheelSpeeds.left * tm_diff
        # r_encoder = self._drivetrain.wheelSpeeds.left * tm_diff
        #
        # if log_it:
        #     logger.info(f"encoder: left: {l_encoder}, right: {r_encoder}")
        #
        # pose = self._field.getRobotPose()
        # x = min(max(0, pose.X()), 17.5)
        # y = min(max(0, pose.Y()), 8)
        # rotation = pose.rotation()
        # pose = self._field.setRobotPose(Pose2d(x, y,rotation))
        # # #
        # # transform = self._drivetrain.calculate(controller_forward_speed, controller_rotation_speed, tm_diff)
        # # self._physics_controller.move_robot(transform)
        #
        # # optional: compute encoder
        # # l_encoder = self.drivetrain.l_position * ENCODER_TICKS_PER_FT
        # # r_encoder = self.drivetrain.r_position * ENCODER_TICKS_PER_FT
        #
        # # FUTURE Update the gyro simulation
        # # -> FRC gyros are positive clockwise, but the returned pose is positive
        # #    counter-clockwise
        # #self.gyro.setAngle(-pose.rotation().degrees())