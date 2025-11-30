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
import time
from typing import Optional

import commands2
import commands2.button
import commands2.cmd
import wpilib
from wpilib import RobotBase, XboxController

from frc_2025 import constants
from frc_2025.commands.holonomicdrive import HolonomicDrive
from frc_2025.subsystems.swervedrive.constants import OIConstants
from frc_2025.subsystems.swervedrive.drivesubsystem import Swerve

# TODO: pathplanner stuff needed?

logger = logging.getLogger(__name__)

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self, robot: RobotBase) -> None:
        # The robot's subsystems
        logger.debug("*** called container __init__")
        self.start_time = time.time()
        self.robot = robot

        self.simulation = RobotBase.isSimulation()

        # The driver's controller
        self.driver_controller = commands2.button.CommandXboxController(constants.kDriverControllerPort)

        # # From the 2025 Java (TODO: Get the JSON files from swerve/neo and update our python code/validate it)
        # filePath = os.path.join(getDeployDirectory(), "swerve/neo")
        # self.robotDrive = SwerveSubsystem(filePath)
        self.robotDrive = self.robot_drive = Swerve(robot)

        # Configure the button bindings and autos
        if isinstance(self.driver_controller, commands2.button.CommandXboxController):
            self.configureButtonBindings_xbox()
        else:
            self.configureButtonBindings_Joystick()

        self.configureAutos()

        self.initialize_dashboard()

        # Configure default command for driving using joystick sticks
        drive_cmd = HolonomicDrive(self,
                                   self.robotDrive,
                                   forwardSpeed=lambda: -self.driver_controller.getRawAxis(XboxController.Axis.kLeftY),
                                   leftSpeed=lambda: -self.driver_controller.getRawAxis(XboxController.Axis.kLeftX),
                                   rotationSpeed=lambda: -self.driver_controller.getRawAxis(
                                       XboxController.Axis.kRightX),
                                   deadband=OIConstants.kDriveDeadband,
                                   fieldRelative=True,
                                   rateLimit=True,
                                   square=True)

        self.robotDrive.setDefaultCommand(drive_cmd)
        #
        # # TODO: Move pathfinding init here so it is ready for autonomous mode
        #

    def set_start_time(self) -> None:  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(self) -> float:  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def elapsed_time(self) -> float:
        return time.time() - self.start_time

    def configureButtonBindings_xbox(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        logger.debug("*** called configureButtonBindings")

        # TODO: Need to reconcile with java cade
        # # driveFieldOrientedDirectAngle      = robotDrive.driveFieldOriented(self.driveDirectAngle)
        # driveFieldOrientedAngularVelocity = self.robotDrive.driveFieldOriented(self.driveAngularVelocity)
        # driveRobotOrientedAngularVelocity = self.robotDrive.driveFieldOriented(self.driveRobotOriented)
        #
        # self.robotDrive.setDefaultCommand(driveFieldOrientedAnglularVelocity)

        self.driver_controller.a().onTrue(commands2.cmd.runOnce(lambda: self.robotDrive.zeroGyro))
        self.driver_controller.y().whileTrue(
            commands2.cmd.runOnce(lambda: self.robotDrive.lock, self.robotDrive).repeatedly())
        self.driver_controller.start().onTrue(commands2.cmd.runOnce(lambda: self.robotDrive.resetGyroToInitial))
        # self.driver_controller.leftBumper().onTrue(driveRobotOrientedAngularVelocity)
        # self.driver_controller.rightBumper().onTrue(driveFieldOrientedAngularVelocity)

    def configureButtonBindings_Joystick(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        pass  # TODO: Not supported at this time. If this is supported, look into places where
        #       XboxController may be used directly (such as in the default drive command
        #       above). Eventually need to abstract this.

    def disablePIDSubsystems(self) -> None:
        """Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup."""

        logger.debug("*** called disablePIDSubsystems")
        self.setMotorBrake(True)

    def setMotorBrake(self, brake: bool) -> None:
        self.robotDrive.setMotorBrake(brake)

    def configureAutos(self):
        self.chosenAuto = wpilib.SendableChooser()
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("trajectory example", self.getAutonomousTrajectoryExample)
        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)
        wpilib.SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getAutonomousLeftBlue(self):
        setStartPose = ResetXY(x=0.783, y=6.686, headingDegrees=+60, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self):
        setStartPose = ResetXY(x=15.777, y=4.431, headingDegrees=-120, drivetrain=self.robotDrive)
        driveForward = commands2.RunCommand(lambda: self.robotDrive.arcadeDrive(xSpeed=1.0, rot=0.0), self.robotDrive)
        stop = commands2.InstantCommand(lambda: self.robotDrive.arcadeDrive(0, 0))

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getAutonomousTrajectoryExample(self) -> commands2.Command:
        command = SwerveTrajectory(
            drivetrain=self.robotDrive,
            speed=+1.0,
            waypoints=[
                (1.0, 4.0, 0.0),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (2.5, 5.0, 0.0),  # next waypoint: x=2.5, y=5.0
                (3.0, 6.5, 0.0),  # next waypoint
                (6.5, 5.0, -90),  # next waypoint
            ],
            endpoint=(6.0, 4.0, -180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True,  # to keep driving onto next command, set =False
        )

        return command

    def getTestCommand(self) -> Optional[commands2.Command]:
        """
        :returns: the command to run in test mode ("test dance") to exercise all subsystems
        """

        # example commands that test drivetrain's motors and gyro (our only subsystem)
        turnRight = AimToDirection(degrees=-45, drivetrain=self.robotDrive, speed=0.25)
        turnLeft = AimToDirection(degrees=45, drivetrain=self.robotDrive, speed=0.25)
        backToZero = AimToDirection(degrees=0, drivetrain=self.robotDrive, speed=0.0)

        command = turnRight.andThen(turnLeft).andThen(backToZero)
        return command

    def initialize_dashboard(self):
        logger.info("*** called initialize_dashboard")
        #
        #  Taken from the FRC2429_2025 project   TODO: what do we need here
        #
        #
        # wpilib.SmartDashboard.putData(MoveLowerArmByNetworkTables(container=self, crank=self.lower_crank))
        # lots of putdatas for testing on the dash
        # COMMANDS FOR GUI (ROBOT DEBUGGING) - 20250224 CJH
        # self.led_mode_chooser = wpilib.SendableChooser()
        # [self.led_mode_chooser.addOption(key, value) for key, value in self.led.modes_dict.items()]  # add all the indicators
        # self.led_mode_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
        #     SetLEDs(container=self, led=self.led, mode=selected_value)))
        #
        # wpilib.SmartDashboard.putData('LED Mode', self.led_mode_chooser)
        #
        # self.led_indicator_chooser = wpilib.SendableChooser()
        #
        # [self.led_indicator_chooser.addOption(key, value) for key, value in self.led.indicators_dict.items()]  # add all the indicators
        # self.led_indicator_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
        #     SetLEDs(container=self, led=self.led, indicator=selected_value)))
        # wpilib.SmartDashboard.putData('LED Indicator', self.led_indicator_chooser)

        # # Arshan's 67 scoring trajectory tests
        # wpilib.SmartDashboard.putData('67 score trajectory L2', FollowTrajectory(container=self, current_trajectory=CustomTrajectory(trajectory.score_waypoint_dict['l2'], list(trajectory.score_waypoint_dict['l2'].keys())[-1]), wait_to_finish=True, ignore_wrist=True))
        # wpilib.SmartDashboard.putData('67 score trajectory L3', FollowTrajectory(container=self, current_trajectory=CustomTrajectory(trajectory.score_waypoint_dict['l3'], list(trajectory.score_waypoint_dict['l3'].keys())[-1]), wait_to_finish=True, ignore_wrist=True))
        # wpilib.SmartDashboard.putData('67 score trajectory L4', FollowTrajectory(container=self, current_trajectory=CustomTrajectory(trajectory.score_waypoint_dict['l4'], list(trajectory.score_waypoint_dict['l4'].keys())[-1]), wait_to_finish=True, ignore_wrist=True))
        #
        # #wpilib.SmartDashboard.putData('67 score trajectory', FollowTrajectory(container=self, current_trajectory=CustomTrajectory(trajectory.score_waypoint_dict[self.robot_state.get_target().value['name']], list(trajectory.score_waypoint_dict[self.robot_state.get_target().value['name']].keys())[-1]), wait_to_finish=True))
        #
        # #.keys()[-1]
        #
        # '''
        # wpilib.SmartDashboard.putData('l3 trajectory', FollowTrajectorya(container=self, current_trajectory=trajectory.trajectory_L3, wait_to_finish=True))
        # wpilib.SmartDashboard.putData('l2 67 score trajectory', FollowTrajectory(container=self, current_trajectory=trajectory.l2_score_67, wait_to_finish=True))
        # wpilib.SmartDashboard.putData('l3 67 score trajectory', FollowTrajectory(container=self, current_trajectory=trajectory.l3_score_67, wait_to_finish=True))
        # wpilib.SmartDashboard.putData('l4 67 score trajectory', FollowTrajectory(container=self, current_trajectory=trajectory.l4_score_67, wait_to_finish=True))'
        # '''
        #
        # # experimental, not used on dash
        # SmartDashboard.putData("Go to 60 deg pid", commands2.cmd.runOnce(lambda: self.pivot.set_goal(math.radians(60), False), self.pivot))
        # SmartDashboard.putData("Go to 90 deg pid", commands2.cmd.runOnce(lambda: self.pivot.set_goal(math.radians(90), False), self.pivot))
        # wpilib.SmartDashboard.putData('SetSuccess', SetLEDs(container=self, led=self.led, indicator=Led.Indicator.kSUCCESS))
        # wpilib.SmartDashboard.putData('MoveElevator', MoveElevator(container=self, elevator=self.elevator, mode='absolute'))
        # wpilib.SmartDashboard.putData('MovePivot', MovePivot(container=self, pivot=self.pivot, mode='absolute'))
        # wpilib.SmartDashboard.putData('SequentialScore', SequentialScoring(container=self))
        # wpilib.SmartDashboard.putData('Move wrist to -90 deg', MoveWrist(container=self, radians=math.radians(-90), timeout=4))
        # wpilib.SmartDashboard.putData('Move wrist to 0 deg', MoveWrist(container=self, radians=math.radians(0), timeout=4))
        # wpilib.SmartDashboard.putData('Move wrist to 90 deg', MoveWrist(container=self, radians=math.radians(90), timeout=4))
        #
        # # commands for pyqt dashboard - please do not remove
        # wpilib.SmartDashboard.putData('MoveElevatorTop', MoveElevator(container=self, elevator=self.elevator, mode='specified', height=constants.ElevatorConstants.k_max_height-0.005 ))
        # wpilib.SmartDashboard.putData('MoveElevatorUp', MoveElevator(container=self, elevator=self.elevator, mode='incremental', height=0.1 ))
        # wpilib.SmartDashboard.putData('MoveElevatorDown', MoveElevator(container=self, elevator=self.elevator, mode='incremental', height=-0.1))
        # wpilib.SmartDashboard.putData('MovePivotUp', MovePivot(container=self, pivot=self.pivot, mode='incremental', angle=10))
        # wpilib.SmartDashboard.putData('MovePivotDown', MovePivot(container=self, pivot=self.pivot, mode='incremental', angle=-10))
        # wpilib.SmartDashboard.putData('MoveWristUp', MoveWrist(container=self, incremental=True, radians=degreesToRadians(30), timeout=0.2))
        # wpilib.SmartDashboard.putData('MoveWristDown', MoveWrist(container=self, incremental=True, radians=degreesToRadians(-30), timeout=0.2))
        # wpilib.SmartDashboard.putData('IntakeOn', RunIntake(container=self, intake=self.intake, value=6, stop_on_end=False))
        # wpilib.SmartDashboard.putData('IntakeOff', RunIntake(container=self, intake=self.intake, value=0, stop_on_end=False))
        # wpilib.SmartDashboard.putData('IntakeReverse', RunIntake(container=self, intake=self.intake, value=-6, stop_on_end=False))
        # wpilib.SmartDashboard.putData('Move climber up', MoveClimber(self, self.climber, 'incremental', math.radians(10)))
        # wpilib.SmartDashboard.putData('Move climber down', MoveClimber(self, self.climber, 'incremental', math.radians(-10)))
        # wpilib.SmartDashboard.putData('CANStatus', CANStatus(container=self))
        # wpilib.SmartDashboard.putData("ResetFlex", Reflash(container=self))
        # wpilib.SmartDashboard.putData('GoToScore', Score(container=self))
        # wpilib.SmartDashboard.putData('GoToStow', GoToStow(container=self))
        # wpilib.SmartDashboard.putData('GoToL1', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L1)).ignoringDisable(True).andThen(GoToReefPosition(self, 1)))
        # wpilib.SmartDashboard.putData('GoToL2', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L2)).ignoringDisable(True).andThen(GoToReefPosition(self, 2)))
        # wpilib.SmartDashboard.putData('GoToL3', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L3)).ignoringDisable(True).andThen(GoToReefPosition(self, 3)))
        # wpilib.SmartDashboard.putData('GoToL4', commands2.InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L4)).ignoringDisable(True).andThen(GoToReefPosition(self, 4)))
        # wpilib.SmartDashboard.putData('Set valid tag IDs', SetValidTags(self, constants.VisionConstants.k_valid_tags))
        # wpilib.SmartDashboard.putData('CalElevatorUp', commands2.InstantCommand(lambda: self.elevator.offset_encoder_position_meters(0.025)).ignoringDisable(True))
        # wpilib.SmartDashboard.putData('CalElevatorDown', commands2.InstantCommand(lambda: self.elevator.offset_encoder_position_meters(-0.025)).ignoringDisable(True))
        # wpilib.SmartDashboard.putData('RecalWrist', RecalibrateWrist(container=self).withTimeout(10))
        # wpilib.SmartDashboard.putData('CalWristUp', commands2.InstantCommand(lambda: self.wrist.offset_encoder_position_degrees(2)).ignoringDisable(True))
        # wpilib.SmartDashboard.putData('CalWristDown', commands2.InstantCommand(lambda: self.wrist.offset_encoder_position_degrees(-2)).ignoringDisable(True))
        # # end pyqt dashboard section

        # # quick way to test all scoring positions from dashboard
        # self.score_test_chooser = wpilib.SendableChooser()
        # [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.targets_dict.items()]  # add all the indicators
        # self.score_test_chooser.onChange(
        #     listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
        #         commands2.cmd.runOnce(lambda: self.robot_state.set_target(target=selected_value))))
        # wpilib.SmartDashboard.putData('RobotScoringMode', self.score_test_chooser)
        #
        # self.auto_chooser = AutoBuilder.buildAutoChooser('')  # this loops through the path planner deploy directory
        # self.auto_chooser.setDefaultOption('1:  Wait *CODE*', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(15)))
        # self.auto_chooser.addOption('2a: Drive 2s Straight *CODE*', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)))
        # self.auto_chooser.addOption('2b: Drive 2s To Driver Station *CODE*', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2.5, field_relative=True)))
        # self.auto_chooser.addOption('3a: 1+2 Right from Center *CODE*', OnePlusTwoRight(self, start='center'))  # the working auto that score three coral on right
        # self.auto_chooser.addOption('3b: 1+2 Right from Right *CODE*', OnePlusTwoRight(self, start='right'))  # the working auto that score three coral on right
        # self.auto_chooser.addOption('4a: 1+2 Left from Center *CODE*', OnePlusTwoLeft(self, start='center'))  # simulated left version of code
        # self.auto_chooser.addOption('4b: 1+2 Left from Left *CODE*', OnePlusTwoLeft(self, start='left'))  # simulated left version of code
        # self.auto_chooser.addOption('5:  1+1 Left? *CODE*', OnePlusOne(self))  #  is there any reason for this?
        # self.auto_chooser.addOption('6:  1+0 l4 auto aim', L4PreloadAutoAim(self))
        # # self.auto_chooser.addOption('7: madtwon', MadtownGlaze(self))
        # # self.auto_chooser.addOption('1+2 trough', OnePlusTwoTrough(self))  # not a real auto
        # wpilib.SmartDashboard.putData('autonomous routines', self.auto_chooser)  #

