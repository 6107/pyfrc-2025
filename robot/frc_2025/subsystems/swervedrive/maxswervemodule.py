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

from typing import Union

import wpilib
from phoenix6.hardware import CANcoder
from rev import SparkMax, SparkFlex, SparkLowLevel, SparkBase
from wpilib import RobotBase
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from frc_2025.subsystems.swervedrive.constants import ModuleConstants, getSwerveDrivingMotorConfig, \
    getSwerveTurningMotorConfig


class MAXSwerveModule:
    def __init__(self,
                 drivingCANId: int,
                 turningCANId: int,
                 chassisAngularOffset: float,
                 driveMotorInverted: bool = False,
                 turnMotorInverted: bool = True,
                 motorControllerType: Union[type(SparkFlex), type(SparkMax)] = SparkFlex,
                 encoder_analog_port: int = -1,
                 turning_encoder_offset: float = 0.0,
                 label: str = ""):
        """
        Constructs a MAXSwerveModule and configures the driving and turning motor,
        encoder, and PID controller. This configuration is specific to the REV
        MAXSwerve Module built with NEOs, SPARKS MAX, and a through-bore Encoder.
        """
        self.label = label
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        self.drivingSparkMax = motorControllerType(drivingCANId, SparkLowLevel.MotorType.kBrushless)
        self.turningSparkMax = motorControllerType(turningCANId, SparkLowLevel.MotorType.kBrushless)

        # Factory reset, so we get the SPARKS MAX to a known state before configuring
        # them. This is useful in case a SPARK MAX is swapped out.
        # TODO: 2026 Support needed:
        # self.drivingSparkMax.configure(getSwerveDrivingMotorConfig(driveMotorInverted),
        #                                SparkBase.ResetMode.kResetSafeParameters,
        #                                SparkBase.PersistMode.kPersistParameters)
        #
        # self.turningSparkMax.configure(getSwerveTurningMotorConfig(turnMotorInverted),
        #                                SparkBase.ResetMode.kResetSafeParameters,
        #                                SparkBase.PersistMode.kPersistParameters)

        # TODO: 2026 Support needed.  Even code below does not worl:
        # self.drivingSparkMax.configure(getSwerveDrivingMotorConfig(driveMotorInverted))
        #
        # self.turningSparkMax.configure(getSwerveTurningMotorConfig(turnMotorInverted))

        # Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        #
        # The turning PID controller on the Spark Max should use its own internal
        # relative encoder as the feedback device for ongoing control, but it is
        # reset to the absolute position periodically or on startup.
        self.drivingEncoder = self.drivingSparkMax.getEncoder()
        self.turningEncoder = self.turningSparkMax.getAbsoluteEncoder()

        self.drivingPIDController = self.drivingSparkMax.getClosedLoopController()
        self.turningPIDController = self.turningSparkMax.getClosedLoopController()

        #  ---------------- ABSOLUTE ENCODER AND PID FOR TURNING  ------------------
        self.absolute_encoder = CANcoder(encoder_analog_port) if encoder_analog_port >= 0 else None

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState.angle = Rotation2d(self.turningEncoder.getPosition())
        self.drivingEncoder.setPosition(0)

    def get_turn_encoder(self):
        # how we invert the absolute encoder if necessary (which it probably isn't in the standard mk4i config)
        position = self.absolute_encoder.get_absolute_position().value
        return -position if ModuleConstants.TURNING_ENCODER_INVERTED else position

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModuleState(self.drivingEncoder.getVelocity(),
                                 Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset))

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        # Apply chassis angular offset to the encoder position to get the position
        # relative to the chassis.
        return SwerveModulePosition(self.drivingEncoder.getPosition(),
                                    Rotation2d(self.turningEncoder.getPosition() - self.chassisAngularOffset))

    def getDesiredState(self):
        return self.desiredState

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.

        """
        if abs(desiredState.speed) < ModuleConstants.MIN_DRIVING_SPEED:
            # if WPILib doesn't want us to move at all, don't bother to bring the wheels back to zero angle yet
            # (causes brownout protection when battery is lower: https://youtu.be/0Xi9yb1IMyA)
            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
            if not inXBrake:
                self.stop()
                return

        # Apply chassis angular offset to the desired state.
        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle + Rotation2d(self.chassisAngularOffset)

        # Optimize the reference state to avoid spinning further than 90 degrees.
        optimizedDesiredState = correctedDesiredState
        SwerveModuleState.optimize(optimizedDesiredState,
                                   Rotation2d(self.turningEncoder.getPosition()))

        # Command driving and turning SPARKS MAX towards their respective setpoints.
        self.drivingPIDController.setReference(optimizedDesiredState.speed,
                                               SparkLowLevel.ControlType.kVelocity)

        self.turningPIDController.setReference(optimizedDesiredState.angle.radians(),
                                               SparkLowLevel.ControlType.kPosition)
        self.desiredState = desiredState

        if RobotBase.isSimulation():
            wpilib.SmartDashboard.putNumberArray(f"{self.label}_target_vel_angle",
                                                 [correctedDesiredState.speed,
                                                  correctedDesiredState.angle.radians()])

            wpilib.SmartDashboard.putNumberArray(f"{self.label}_actual_vel_angle",
                                                 [self.drivingEncoder.getVelocity(),
                                                  self.turningEncoder.getPosition()])

            wpilib.SmartDashboard.putNumberArray(f"{self.label}_volts",
                                                 [self.drivingSparkMax.getAppliedOutput(),
                                                  self.turningSparkMax.getAppliedOutput()])

    def stop(self):
        """
        Stops the module in place to conserve energy and avoid unnecessary brownouts
        """
        self.drivingPIDController.setReference(0, SparkLowLevel.ControlType.kVelocity)
        self.turningPIDController.setReference(self.turningEncoder.getPosition(),
                                               SparkLowLevel.ControlType.kPosition)
        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """
        Zeroes all the SwerveModule encoders.
        """
        if self.absolute_encoder is not None:
            # Read the absolute position from the CANCoder
            # TODO: Units might need conversion (rotations, degrees, or radians)
            position = self.absolute_encoder.get_absolute_position().value
        else:
            position = 0

        self.turningEncoder.setPosition(position)
        self.drivingEncoder.setPosition(0)

