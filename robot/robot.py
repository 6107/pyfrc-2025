#!/usr/bin/env python3
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
import sys
from typing import Optional

import wpilib
from commands2 import TimedCommandRobot, CommandScheduler
from commands2.command import Command
from phoenix6.hardware.talon_fx import TalonFX
from rev import SparkFlex, SparkMax
from wpilib import Timer, RobotBase, DriverStation, Field2d, SmartDashboard

from frc_2025 import constants
from frc_2025.robotcontainer import RobotContainer
# from util.telemetry import Telemetry
from version import VERSION

# Setup Logging
logger = logging.getLogger(__name__)

"""
The VM is configured to automatically run this class, and to call the functions corresponding to
each mode, as described in the TimedRobot documentation. If you change the name of this class or
the package after creating this project, you must also update the build.gradle file in the
project.
"""
class MyRobot(TimedCommandRobot):
    """
    Our default robot class

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    def __init__(self):
        # Initialize our base class, choosing the default scheduler period
        super().__init__()

        logger.debug("*** called MyRobot __init__")
        self._counter = 0  # Updated on each periodic call. Can be used to logging/smartdashboard updates

        self._container: Optional[RobotContainer] = None
        self.autonomousCommand: Optional[Command] = None

        self.disabledTimer: Timer = Timer()
        self.autonomousCommand: Optional[Command] = None

        self.holding_alge = False
        self.autonomousCommand = None
        self.field: Optional[wpilib.Field2d] = None

        # Visualization and pose support
        self.match_started = False  # Set true on Autonomous or Teleop init

    @property
    def container(self) -> RobotContainer:
        return self._container

    @property
    def counter(self) -> int:
        return self._counter

    @property
    def elevator(self) -> TalonFX:
        return self.container.elevator

    @property
    def alge_roller(self) -> SparkFlex:
        return self.container.alge_roller

    @property
    def alge_rotation(self) -> SparkMax:
        return self.container.alge_rotation

    @property
    def intake_left(self) -> SparkMax:
        return self.container.intake_left

    @property
    def intake_right(self) -> SparkMax:
        return self.container.intake_right

    @property
    def intake_extend(self) -> SparkMax:
        return self.container.intake_extend

    @property
    def c_alge_rotation(self) -> SparkMax:
        return self.container.c_alge_rotation

    @property
    def c_intake_extend(self) -> SparkMax:
        return self.container.c_intake_extend

    # @tracer.start_as_current_span("robotInit")
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        if RobotBase.isSimulation():
            logger.setLevel(logging.INFO)

            # If this is a simulation, we need to silence joystick warnings
            logger.warning("Simulation detected. Silencing annoying JoyStick warnings")
            DriverStation.silenceJoystickConnectionWarning(True)
        else:
            logger.setLevel(logging.ERROR)

        logging.getLogger("wpilib").setLevel(logging.DEBUG)
        logging.getLogger("commands2").setLevel(logging.DEBUG)

        version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
        logger.info(f"Python: {version}, Software Version: {VERSION}")

        # Set up our playing field. May get overwritten if simulation is running or if we
        # support vision based odometry
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self._container = RobotContainer(self)
        self.disabledTimer = wpilib.Timer()

    def robotPeriodic(self) -> None:
        """
        Periodic code for all modes should go here.

        This function is called each time a new packet is received from the driver
        station. Default period is 20 mS.
        """
        logger.debug(f"called robotPeriodic: enabled: {self.isEnabled()}")

        self._counter += 1

        if self.isEnabled():
            self.container.robotDrive.periodic()

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        # CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """
        Initialization code for disabled mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters disabled mode.
        """
        logger.debug("called disabledInit")
        self.container.disablePIDSubsystems()
        self.disabledTimer.reset()
        self.disabledTimer.start()

    def disabledPeriodic(self) -> None:
        """
        Periodic code for disabled mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in disabled
        mode.
        """
        logger.debug("called disabledPeriodic")

        if self.disabledTimer.hasElapsed(constants.WHEEL_LOCK_TIME):
            self.container.setMotorBrake(False)
            self.disabledTimer.stop()
            self.disabledTimer.reset()

        # Validate who we are working for
        if not self.match_started:
            self.container.check_alliance()

    def disabledExit(self) -> None:
        """
        Exit code for disabled mode should go here.

        Users should override this method for code which will be called each time
        the robot exits disabled mode.
        """
        logger.info("*** disabledExit: entry")
        self.disabledTimer.stop()
        self.disabledTimer.reset()

    def autonomousInit(self) -> None:
        """
        Initialization code for autonomous mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters autonomous mode.
        """
        self.container.set_start_time()

        # Stop what we are doing...
        self.container.setMotorBrake(True)

        # Validate who we are working for. This may not be valid until autonomous or teleop init
        if not self.match_started:
            self.container.check_alliance()
            self.match_started = True

        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """
        Periodic code for autonomous mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in
        autonomous mode.
        """
        logger.debug("*** called autonomousPeriodic")

    def autonomousExit(self) -> None:
        """
        Exit code for autonomous mode should go here.

        Users should override this method for code which will be called each time
        the robot exits autonomous mode.
        """
        logger.info("*** autonomousExit: entry")

        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopInit(self) -> None:
        """
        Initialization code for teleop mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters teleop mode.
        """
        logger.info("*** called teleopInit")

        self.container.set_start_time()

        # Stop what we are doing...
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        else:
            CommandScheduler.getInstance().cancelAll()

        # Validate who we are working for. This may not be valid until autonomous or teleop init
        if not self.match_started:
            self.container.check_alliance()
            self.match_started = True

    def teleopPeriodic(self) -> None:
        """
        Periodic code for teleop mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in teleop
        mode.
        """
        # Intake wheel control logic
        logger.debug("*** called teleopPeriodic")

        # TODO: Move these all to commands and come up with some generic classes
        #       like 'Intake', 'Shooter', 'Elevator', ...

        # # TODO: As we implement and understand the following, document what we are
        # #       doing here.
        # if self.holding_alge:
        #     self.intake_left.set(constants.D_ALGE_HOLD_SPEED)
        #     self.intake_right.set(-constants.D_ALGE_HOLD_SPEED)
        #
        # shooter = self.container.controller_shooter
        #
        # intake_coral = shooter.getLeftBumperButton()
        # if intake_coral:
        #     self.intake_left.set(constants.D_CORAL_INTAKE_SPEED)
        #     self.intake_right.set(-constants.D_CORAL_INTAKE_SPEED)
        #     self.holding_alge = False
        #
        # intake_alge = shooter.getLeftTriggerAxis() >= 0.35
        # if intake_alge:
        #     self.intake_left.set(constants.D_ALGE_INTAKE_SPEED)
        #     self.intake_right.set(-constants.D_ALGE_INTAKE_SPEED)
        #     self.holding_alge = True
        #
        # b_shoot = shooter.getRightTriggerAxis() >= 0.35
        # if b_shoot:
        #     self.intake_left.set(constants.D_SHOOT_SPEED)
        #     self.intake_right.set(-constants.D_SHOOT_SPEED)
        #     self.holding_alge = False
        #
        # # Elevator control logic
        # pos_l0 = shooter.getRightBumperButtonPressed()
        # pos_l1 = shooter.getAButtonPressed()
        # pos_l2 = shooter.getXButtonPressed()
        # pos_l3 = shooter.getYButtonPressed()
        #
        # if pos_l0:
        #     self.elevator.set_control(constants.EL_POS_L0)
        #
        # if pos_l1:
        #     self.elevator.set_control(constants.EL_POS_L1)
        #
        # if pos_l2:
        #     self.elevator.set_control(constants.EL_POS_L2)
        #
        # if pos_l3:
        #     self.elevator.set_control(constants.EL_POS_L3)
        #
        # pos_intake = shooter.getBButtonPressed()
        # if pos_intake:
        #     self.elevator.set_control(constants.EL_POS_IN)
        #
        # # TODO: Support additional subsystems
        # # intake_extend = math.fabs(shooter.getLeftY())
        # # # Intake extender logic
        # # if intake_extend >= constants.DEADBAND:
        # #     self.intake_extend.setReference(intake_extend * constants.I_INTAKE_EXTEND_MAX,
        # #                                     SparkBase.ControlType.kPosition)
        # # else:
        # #     self.intake_extend.setReference(0, SparkBase.ControlType.kPosition)
        # #
        # # shoot     = shooter.getRightTriggerAxis() >= 0.35
        # # alge_grab = shooter.getRightStickButton()
        # #
        # # # Alge intake control logic
        # # if alge_grab:
        # #     self.alge_roller.set(constants.D_ALGE_GRABBER_GRAB)
        # #     self.alge_rotation.setReference(constants.I_ALGE_ROTATION_OUT, SparkBase.ControlType.kPosition)
        # #
        # # elif shoot:
        # #     self.alge_roller.set(constants.D_ALGE_GRABBER_SHOOT)
        # #     self.alge_rotation.setReference(constants.I_ALGE_ROTATION_IN, SparkBase.ControlType.kPosition)
        # #
        # # else:
        # #     self.alge_roller.set(constants.D_ALGE_GRABBER_HOLD)
        # #     self.alge_rotation.setReference(constants.I_ALGE_ROTATION_IN, SparkBase.ControlType.kPosition)

    def teleopExit(self) -> None:
        """
        Exit code for teleop mode should go here.

        Users should override this method for code which will be called each time
        the robot exits teleop mode.
        """
        pass

    def testInit(self) -> None:
        """
        Initialization code for test mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters test mode.
        """
        logger.info("*** called testInit")
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """
        Periodic code for test mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in test
        mode.
        """
        logger.info("*** called testPeriodic")
        pass

    def testExit(self):
        """
        Exit code for test mode should go here.

        Users should override this method for code which will be called each time
        the robot exits test mode.
        """
        logger.info("*** called testExit")
        pass

    def _simulationInit(self) -> None:
        """
        Robot-wide simulation initialization code should go here.

        Users should override this method for default Robot-wide simulation
        related initialization which will be called when the robot is first
        started. It will be called exactly one time after RobotInit is called
        only when the robot is in simulation.
        """
        logger.info("*** _simulationInit: entry")

    def _simulationPeriodic(self):
        """
        Periodic simulation code should go here.

        This function is called in a simulated robot after user code executes.
        """
        logger.info("*** _simulationPeriodic: entry")
