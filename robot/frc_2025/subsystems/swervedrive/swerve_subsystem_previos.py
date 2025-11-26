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

import logging

import commands2
from commands2.cmd import run

from pathplannerlib.config import RobotConfig
from pathplannerlib.path import PathConstraints
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.util.swerve import SwerveSetpointGenerator, SwerveSetpoint
from pathplannerlib.auto import PathPlannerAuto

from wpilib import PWMSparkMax, Encoder, DriverStation, SmartDashboard, Field2d
from wpilib.drive import DifferentialDrive
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Kinematics, SwerveDrive4Odometry, SwerveModuleState
from wpimath.geometry import Pose2d, Translation2d, Rotation2d, Rotation3d
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.trajectory import Trajectory

from wpilib.interfaces import MotorController
from wpilib.drive import SwerveDrive4Controller


from frc_2025 import constants

logger = logging.getLogger(__name__)

"""
To create an FRC swerve drive subsystem in Python, you will need to create a
SwerveDrive class that uses wpimath.kinematics.SwerveDrive4Kinematics (or the
appropriate version for your number of modules) to calculate the required
states for each swerve module. The subsystem will manage four independent
swerve modules, each requiring a speed motor and a steering motor, along
with associated encoders and the ability to track angle. An example project
structure and a helpful starting point can be found in the official robotpy
examples.

"""
# A single swerve module is the core component.
class SwerveModule:
    def __init__(self, drive_motor: MotorController, steer_motor: MotorController, encoder: Encoder):
        self.driveMotor = drive_motor
        self.steerMotor = steer_motor
        self.encoder = encoder

    def setState(self, state: SwerveModuleState):
        # Set the drive motor to the target speed
        self.driveMotor.set(state.speed)

        # Set the steering motor to the target angle
        self.steerMotor.setAngle(state.angle)

    def getPosition(self) -> SwerveModulePosition:
        # Get the position from the encoder
        return SwerveModulePosition(self.encoder.getDistance(), self.encoder.getDistance())


class SwerveDriveSubsystem(commands2.Subsystem):
    # Creates a new DriveSubsystem
    def old__init__(self, directory: str):
        super().__init__()
        self.directory: str = directory
        self._swerveDrive = None

        # Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        # SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        # try
        # {
        #   swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.MAX_SPEED,
        #                                                               new Pose2d(new Translation2d(Meter.of(1),
        #                                                                                            Meter.of(4)),
        #                                                                          Rotation2d.fromDegrees(0)));
        #   // Alternative method if you don't want to supply the conversion factor via JSON files.
        #   // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
        # } catch (Exception e)
        # {
        #   throw new RuntimeException(e);
        # }
        # Heading correction should only be used while controlling  the robot via angle.
        self._swerveDrive.setHeadingCorrection(false)

        # Disables cosine compensation for simulations since it  causes discrepancies not seen in real life.
        self._swerveDrive.setCosineCompensator(false)

        # Correct for skew that gets worse as angular  velocity increases. Start with a coefficient of 0.1.
        self._swerveDrive.setAngularVelocityCompensation(True, True, 0.1)

        # Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        self._swerveDrive.setModuleEncoderAutoSynchronize(False, 1)

        ## Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
        # //    swerveDrive.pushOffsetsToEncoders(); //

    def __init__(self):
        super().__init__()

        """
        The core of the system is a collection of four swerve modules, each 
        consisting of a motor for driving the wheel and a motor for steering 
        it. You'll also need to track the angle of each module, typically 
        with an encoder.
        """
        # Initialize the four swerve modules
        self.frontLeft = SwerveModule(
            rev.SparkMax(1, rev.CANSparkMax.MotorType.kBrushless),
            rev.SparkMax(2, rev.CANSparkMax.MotorType.kBrushless),
            wpilib.Encoder(0, 1)
        )
        self.frontRight = SwerveModule(
            rev.SparkMax(3, rev.CANSparkMax.MotorType.kBrushless),
            rev.SparkMax(4, rev.CANSparkMax.MotorType.kBrushless),
            wpilib.Encoder(2, 3)
        )
        self.backLeft = SwerveModule(
            rev.SparkMax(5, rev.CANSparkMax.MotorType.kBrushless),
            rev.SparkMax(6, rev.CANSparkMax.MotorType.kBrushless),
            wpilib.Encoder(4, 5)
        )
        self.backRight = SwerveModule(
            rev.SparkMax(7, rev.CANSparkMax.MotorType.kBrushless),
            rev.SparkMax(8, rev.CANSparkMax.MotorType.kBrushless),
            wpilib.Encoder(6, 7)
        )
        """
        Use wpimath.kinematics.SwerveDrive4Kinematics to define the physical 
        layout of your robot's modules (e.g., frontLeftLocation, 
        frontRightLocation). This object will handle the calculations to 
        translate desired robot chassis speeds into individual module states.
        """
        # Create the SwerveDriveKinematics object
        self.kinematics = SwerveDriveKinematics(
            Translation2d(1, 1),  # Front Left module offset
            Translation2d(1, -1),  # Front Right module offset
            Translation2d(-1, 1),  # Back Left module offset
            Translation2d(-1, -1)  # Back Right module offset
        )

        # Create the SwerveDriveOdometry object
        self.odometry = SwerveDriveOdometry(
            self.kinematics,
            Rotation2d(0),  # Initial gyro heading
            self.frontLeft.getPosition(),
            self.frontRight.getPosition(),
            self.backLeft.getPosition(),
            self.backRight.getPosition()
        )
        # Initialize the field and robot visualization
        self.field = Field2d()
        self.field.getObject('robot').setPose(self.odometry.getPose())


    def periodic(self):
        """ Called periodically by the scheduler """

        # TODO: Update odometry in the periodic method


        logger.info("*** called periodic")
        wpilib.SmartDashboard.putNumber("Motor Speed", self._swerveDrive.get())
        wpilib.SmartDashboard.putNumber("Encoder Position", self._swerveDrive.getPosition())
        wpilib.SmartDashboard.putBoolean("Is Motor Alive", True)

    def getAutonomousCommand(self, pathName: str) -> commands2.cmd:
        """
        Get the path follower with events.

        :param pathName:  PathPlanner path name
        :return:  {@link AutoBuilder#followPath(PathPlannerPath)} path command
        """
        # Create a path following command using AutoBuilder. This will also trigger event markers.
        #
        # TODO: Big note from the path planner document below.  We need to do this at init.
        #
        #   This method loads the auto when it is called, however, it is recommended
        #   to first load your paths/autos when code starts, then return the
        #   pre-loaded auto/path
        #
        logger.info("*** called getAutonomousCommand")
        return PathPlannerAuto(pathName)

    def driveToPose(self, pose: Pose2d) -> commands2.cmd:
        """
        Use PathPlanner Path finding to go to a point on the field.

        :param pose: Target {@link Pose2d} to go to
        :return: PathFinding command
        """
        logger.info("*** called driveToPose")
        # Create the constraints to use while pathfinding
        constraints: PathConstraints = PathConstraints(self._swerveDrive.getMaximumChassisVelocity(),        # Max linear velocity
                                                       4.0,                                                  # Max Linear acceleration
                                                       self._swerveDrive.getMaximumChassisAngularVelocity(), # Max angular velocity
                                                       Units.degreesToRadians(720))                          # Max angular acceleration

        # Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(pose,
                                          constraints,
                                          edu.wpi.first.units.Units.MetersPerSecond.of(0)) # Goal end velocity in meters/sec

    def driveWithSetpointGenerator(self, robotRelativeChassisSpeed: ChassisSpeeds) -> commands2.cmd:
        """
        Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.

        :param robotRelativeChassisSpeed: Robot relative {@link ChassisSpeeds} to achieve.
        :return: {@link Command} to run.
        """
        logger.info("*** called driveWithSetpointGenerator")
        setpointGenerator: SwerveSetpointGenerator = SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                                                                             self._swerveDrive.getMaximumChassisAngularVelocity())
        prevSetpoint: SwerveSetpoint =  SwerveSetpoint(self._swerveDrive.getRobotVelocity(),
                                                       self._swerveDrive.getStates(),
                                                       DriveFeedforwards.zeros(swerveDrive.getModules().length))
        previousTime: 'AtomicReference.Double' = AtomicReference()

        # return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
        #             () -> {
        #               double newTime = Timer.getFPGATimestamp();
        #               SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
        #                                                                               robotRelativeChassisSpeed.get(),
        #                                                                               newTime - previousTime.get());
        #               swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
        #                                 newSetpoint.moduleStates(),
        #                                 newSetpoint.feedforwards().linearForces());
        #               prevSetpoint.set(newSetpoint);
        #               previousTime.set(newTime);
        #
        #             });
        # }


    def driveWithSetpointGeneratorFieldRelative(self, fieldRelativeSpeeds: ChassisSpeeds) -> commands2.cmd:
        """
        Drive with 254's Setpoint generator; port written by PathPlanner.

        :param fieldRelativeSpeeds: Field-Relative {@link ChassisSpeeds}
        :return: Command to drive the robot using the setpoint generator.
        """
        logger.info("*** called driveWithSetpointGeneratorFieldRelative")
        try:
            # return driveWithSetpointGenerator(() -> {
            # return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());})
            pass

        except Exception as e:
            DriverStation.reportError(str(e), true)

        return commands2.cmd.none()

    def sysIdDriveMotorCommand(self) -> commands2.cmd:
        """
        Command to characterize the robot drive motors using SysId

        :return: SysId Drive Command
        """
        # TODO :Need to look following up to see if correct
        logger.info("*** called sysIdDriveMotorCommand")

        routine = SwerveDriveTest.setDriveSysIdRoutine(Config(), self, self._swerveDrive,
                                                       12, true)
        return SwerveDriveTest.generateSysIdCommand(routine, 3.0, 5.0, 3.0)

    def sysIdAngleMotorCommand(self) -> commands2.cmd:
        """
        Command to characterize the robot angle motors using SysId

        :return: SysId Angle Command
        """
        # TODO :Need to look following up to see if correct
        logger.info("*** called sysIdAngleMotorCommand")
        routine = SwerveDriveTest.setAngleSysIdRoutine(Config(), self, self._swerveDrive)
        return SwerveDriveTest.generateSysIdCommand(routine,
                                                    3.0, 5.0, 3.0)

    def centerModulesCommand(self) -> commands2.cmd:
        """
        Returns a Command that centers the modules of the SwerveDrive subsystem

        :return:  a Command that centers the modules of the SwerveDrive subsystem
        """
        # return run(() -> Arrays.asList(swerveDrive.getModules())
        #                    .forEach(it -> it.setAngle(0.0)));
        logger.info("*** called centerModulesCommand")
        pass

    def driveToDistanceCommand(self, distanceInMeters: float, speedInMetersPerSecond: float) -> commands2.cmd:
        """
        Returns a Command that drives the swerve drive to a specific distance at a given speed.

        :param distanceInMeters:  the distance to drive in meters
        :param speedInMetersPerSecond:  the speed at which to drive in meters per second
        :return:  a Command that drives the swerve drive to a specific distance at a given speed
        """
        logger.info("*** called driveToDistanceCommand")
        fixed_translation = Translation2d(0, 0)

        def distance_achieved() -> bool:
            self.getPose().getTranslation().getDistance(fixed_translation > distanceInMeters)

        return run(lambda: self.drive(ChassisSpeeds(speedInMetersPerSecond,
                                                    0,
                                                    0))).until(distance_achieved())

    def replaceSwerveModuleFeedforward(self, kS: float, kV: float, kA: float) -> None:
        """
        Replaces the swerve module feedforward with a SimpleMotorFeedforwardMeters object.

        :param kS: the static gain of the feedforward
        :param kV: the velocity gain of the feedforward
        :param kA: the acceleration gain of the feedforward
        """
        logger.info("*** called replaceSwerveModuleFeedforward")

        # TODO: SimpleMotorFeedforwardMeters also takes an optioa dt (delta time)


        self._swerveDrive.replaceSwerveModuleFeedforward(SimpleMotorFeedforwardMeters(kS, kV, kA))

    # TODO: Next two methods are both 'driveCommand' and differ in just the last argument
    def driveCommand(self, translationX: 'DoubleSupplier', translationY: 'DoubleSupplier',
                     angularRotationX: 'DoubleSupplier') -> commands2.cmd:
        """
        Command to drive the robot using translative values and heading as angular velocity.

        :param translationX: Translation in the X direction. Cubed for smoother controls.
        :param translationY: Translation in the Y direction. Cubed for smoother controls.
        :param angularRotationX: Angular velocity of the robot to set. Cubed for smoother controls.
        :return: Drive command
        """
        logger.info("*** called driveCommand-1")
        # Make the robot move
        max_chassis_velocity = self._swerveDrive.getMaximumChassisVelocity()
        max_angular_velocity = self._swerveDrive.getMaximumChassisAngularVelocity()

        translation = Translation2d(translationX.getAsDouble() * max_chassis_velocity,
                                    translationY.getAsDouble() * max_chassis_velocity)
        scaled_tranlation = SwerveMath.scaleTranslation(translation, 0.8)

        return run(lambda: self._swerveDrive.drive(scaled_tranlation,
                                                   Math.pow(angularRotationX.getAsDouble(), 3) * max_angular_velocity,
                                                   True,
                                                   False))

    def driveCommand(self, translationX: 'DoubleSupplier', translationY: 'DoubleSupplier',
                     headingX: 'DoubleSupplier', headingY: 'DoubleSupplier') -> commands2.cmd:
        """
        Command to drive the robot using translative values and heading as a setpoint.

        :param translationX: Translation in the X direction. Cubed for smoother controls.
        :param translationY: Translation in the Y direction. Cubed for smoother controls.
        :param headingX: Heading X to calculate angle of the joystick.
        :param headingY: Heading Y to calculate angle of the joystick
        :return: Drive command
        """
        logger.info("*** called driveCommand-2")
        # # self._swerveDrive.setHeadingCorrection(true); # Normally you would want heading correction for this kind of control.

        def drive_cmd():
            scaled_inputs: Translation2d = SwerveMath.scaleTranslation(Translation2d(translationX.getAsDouble(),
                                                                                     translationY.getAsDouble()), 0.8)
            # Make the robot move
            chassis_speed = self.swerveDrive.swerveController.getTargetSpeeds(scaled_inputs.getX(), scaled_inputs.getY(),
                                                                              headingX.getAsDouble(),
                                                                              headingY.getAsDouble(),
                                                                              swerveDrive.getOdometryHeading().getRadians(),
                                                                              swerveDrive.getMaximumChassisVelocity())
            self.driveFieldOriented(chassis_speed)

        return run(drive_cmd)

    def drive(self, translation: Translation2d, rotation: float, fieldRelative: bool) -> None:
        """
        The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and
        calculates and commands module states accordingly. Can use either open-loop or closed-loop velocity control for
        the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is used.

        :param translation: {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
                            second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
                            torwards port (left).  In field-relative mode, positive x is away from the alliance wall
                            (field North) and positive y is torwards the left wall when looking through the driver station
                            glass (field West).
        :param rotation: Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
                        relativity
        :param fieldRelative: Drive mode.  True for field-relative, false for robot-relative
        """
        logger.info("*** called drive")
        # Open loop is disabled since it shouldn't be used most of the time.
        self._swerveDrive.drive(translation, rotation, fieldRelative, false)

    def driveFieldOriented(self, velocity: ChassisSpeeds) -> None:
        """
        Drive the robot given a chassis field oriented velocity

        :param velocity: Velocity according to the field
        """
        logger.info("*** called driveFieldOriented")
        self.driveFieldOriented(velocity)

    def driveFieldOriented(self, velocity: ChassisSpeeds) -> commands2.cmd:
        """
        Drive the robot given a chassis field oriented velocity

        :param velocity: Velocity according to the field.
        """
        logger.info("*** called driveFieldOriented")
        return Run(lambda: self._swerveDrive.driveFieldOriented(velocity.get()))

    def drive(self, velocity: ChassisSpeeds) -> None:
        """
        Drive according to the chassis robot oriented velocity.

        :param velocity: velocity Robot oriented {@link ChassisSpeeds}
        """
        logger.info("*** called drive")
        self._swerveDrive.drive(velocity)

    def getKinematics(self) -> SwerveDrive4Kinematics:
        """
        Get the swerve drive kinematics object

        :return: {@link SwerveDriveKinematics} of the swerve drive
        """
        logger.info("*** called getKinematics")
        return self._swerveDrive.kinematics

    def resetOdometry(self, initialHolonomicPose: Pose2d) -> None:
        """
        Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
        method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
        keep working.

        :param initialHolonomicPose: The pose to set the odometry to
        """
        logger.info("*** called resetOdometry")
        # self._swerveDrive.resetOdometry(initialHolonomicPose)
        self.odometry.resetPosition(pose, self.gyro.getRotation2d())

    def getPose(self) -> Pose2d:
        """
        Gets the current pose (position and rotation) of the robot, as reported by odometry.

        :return: The robot's pose
        """
        logger.info("*** called getPose")
        # return self._swerveDrive.getPose()
        return self.odometry.getPose()

    def setChassisSpeeds(self, chassisSpeeds: ChassisSpeeds) -> None:
        """
        Set chassis speeds with closed-loop velocity control.

        :param chassisSpeeds: Chassis Speeds to set.
        """
        logger.info("*** called setChassisSpeeds")
        self._swerveDrive.setChassisSpeeds(chassisSpeeds)

    def postTrajectory(self, trajectory: Trajectory) -> None:
        """
        Post the trajectory to the field.

        :param trajectory: The trajectory to post.
        """
        logger.info("*** called postTrajectory")
        self._swerveDrive.postTrajectory(trajectory)

    def zeroGyro(self) -> None:
        """
        Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
        """
        logger.info("*** called zeroGyro")
        self._swerveDrive.zeroGyro()

    def resetGyroToInitial(self) -> None:
        """
        Resets the gyro angle to the initial heading.
        """
        logger.info("*** called resetGyroToInitial")
        self.zeroGyro()
        self._swerveDrive.setGyroOffset(Rotation3d())

    def isRedAlliance(self) -> bool:
        """
        Checks if the alliance is red, defaults to false if alliance isn't available

        :return: true if the red alliance, false if blue. Defaults to false if none is available.
        """
        logger.info("*** called isRedAlliance")
        alliance = DriverStation.getAlliance()
        return alliance.isPresent() if alliance.get() == DriverStation.Alliance.Red else False

    def zeroGyroWithAlliance(self) -> None:
        """
        This will zero (calibrate) the robot to assume the current position is facing forward

        If red alliance rotate the robot 180 after the drivebase zero command
        """
        logger.info("*** called zeroGyroWithAlliance")
        if self.isRedAlliance():
            self.zeroGyro()

            # Set the pose 180 degrees
            self.resetOdometry(Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)))
        else:
            self.zeroGyro()

    def setMotorBrake(self, brake: bool) -> None:
        """
        Sets the drive motors to brake/coast mode.

        :param brake: brake True to set motors to brake mode, false for coast.
        """
        logger.info("*** called setMotorBrake")
        self._swerveDrive.setMotorIdleMode(brake)

    def getHeading(self) -> Rotation2d:
        """
        Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
        Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().

        :return: The yaw angle
        """
        logger.info("*** called getHeading")
        return self.getPose().getRotation()

    def getTargetSpeeds(self, xInput: float, yInput: float, headingX: float, headingY: float) -> ChassisSpeeds:
        """
        Get the chassis speeds based on controller input of 2 joysticks. One for speeds in which direction. The other for
        the angle of the robot.

        :param xInput: X joystick input for the robot to move in the X direction.
        :param yInput: Y joystick input for the robot to move in the Y direction.
        :param headingX: X joystick which controls the angle of the robot.
        :param headingY: Y joystick which controls the angle of the robot.

        :return: {@link ChassisSpeeds} which can be sent to the Swerve Drive.
        """
        logger.info("*** called getTargetSpeeds")
        scaled_inputs: Translation2d = SwerveMath.cubeTranslation(Translation2d(xInput, yInput))

        return self._swerveDrive.swerveController.getTargetSpeeds(scaled_inputs.getX(),
                                                                  scaled_inputs.getY(),
                                                                  headingX,
                                                                  headingY,
                                                                  getHeading().getRadians(),
                                                                  Constants.MAX_SPEED)

    def getTargetSpeeds(self, xInput: float,  yInput: float, angle: Rotation2d) -> ChassisSpeeds:
        """
        Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot at an offset of
        90deg.

        :param xInput: X joystick input for the robot to move in the X direction.
        :param yInput: Y joystick input for the robot to move in the Y direction.
        :param angle: The angle in as a {@link Rotation2d}.

        :return: @link ChassisSpeeds} which can be sent to the Swerve Drive.
        """
        logger.info("*** called getTargetSpeeds")
        scaled_inputs: Translation2d = SwerveMath.cubeTranslation(Translation2d(xInput, yInput))

        return self._swerveDrive.swerveController.getTargetSpeeds(scaled_inputs.getX(),
                                                                  scaled_inputs.getY(),
                                                                  angle.getRadians(),
                                                                  getHeading().getRadians(),
                                                                  Constants.MAX_SPEED)

    def getFieldVelocity(self) -> ChassisSpeeds:
        """
        Gets the current field-relative velocity (x, y and omega) of the robot

        :return: A ChassisSpeeds object of the current field-relative velocity
        """
        logger.info("*** called getFieldVelocity")
        return self._swerveDrive.getFieldVelocity()

    def getRobotVelocity(self) -> ChassisSpeeds:
        """
        Gets the current velocity (x, y and omega) of the robot

        :return:  A {@link ChassisSpeeds} object of the current velocity
        """
        logger.info("*** called getRobotVelocity")
        return self._swerveDrive.getRobotVelocity()

    def getSwerveController(self) -> SwerveController:
        """
        Get the {@link SwerveController} in the swerve drive.

        :return: {@link SwerveController} from the {@link SwerveDrive}.
        """
        logger.info("*** called getSwerveController")
        return self._swerveDrive.swerveController

    def getSwerveDriveConfiguration(self) -> SwerveDriveConfiguration:
        """
        Get the {@link SwerveDriveConfiguration} object.

        :return: The {@link SwerveDriveConfiguration} fpr the current drive.
        """
        logger.info("*** called getSwerveDriveConfiguration")
        return self._swerveDrive.swerveDriveConfiguration

    def lock(self) -> None:
        """
        Lock the swerve drive to prevent it from moving.
        """
        logger.info("*** called lock")
        self._swerveDrive.lockPose()

    def getPitch(self) -> Rotation2d:
        """
        Gets the current pitch angle of the robot, as reported by the imu.

        :return: The heading as a {@link Rotation2d} angle
        """
        logger.info("*** called getPitch")
        return self._swerveDrive.getPitch()

    def addFakeVisionReading(self) -> None:
        """
        Add a fake vision reading for testing purposes.
        """
        logger.info("*** called addFakeVisionReading")
        self._swerveDrive.addVisionMeasurement(Pose2d(3, 3, Rotation2d.fromDegrees(65)),
                                              Timer.getFPGATimestamp())

    def getSwerveDrive(self) -> SwerveDrive:
        """
        Gets the swerve drive object.

        :return: {@link SwerveDrive}
        """
        logger.info("*** called getSwerveDrive")
        return self._swerveDrive

