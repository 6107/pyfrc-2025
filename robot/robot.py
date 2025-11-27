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
import math
import sys
from typing import Optional

import wpilib
from commands2 import TimedCommandRobot, CommandScheduler
from commands2.command import Command
from phoenix6.hardware.talon_fx import TalonFX
from rev import SparkFlex, SparkMax, SparkLowLevel, SparkClosedLoopController
from wpilib import Timer, XboxController, RobotBase, DriverStation

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

        logger.info("*** called __init__")
        self._counter = 0

        self._container: Optional[RobotContainer] = None
        self.autonomousCommand: Optional[Command] = None

        # TODO: Once working in SIM, move the following into the robot container
        #       and subsystem as appropriate
        self.elevator: TalonFX = TalonFX(10)    # Device ID is 10 (as configured in the Phoenix Tuner)
        self.alge_roller: SparkFlex = SparkFlex(11, SparkLowLevel.MotorType.kBrushless)   # Device ID 11
        self.alge_rotation: SparkMax = SparkMax(12, SparkLowLevel.MotorType.kBrushless)   # Device ID 12

        self.c_alge_rotation: SparkClosedLoopController = self.alge_rotation.getClosedLoopController()

        self.intake_left: SparkMax = SparkMax(13, SparkLowLevel.MotorType.kBrushless)   # Device ID 13
        self.intake_right: SparkMax = SparkMax(14, SparkLowLevel.MotorType.kBrushless)   # Device ID 14
        self.intake_extend: SparkMax = SparkMax(15, SparkLowLevel.MotorType.kBrushless)   # Device ID 15

        self.c_intake_extend: SparkClosedLoopController = self.intake_extend.getClosedLoopController()

        self.controller_shooter: XboxController = XboxController(1) # On USB-port 1

        self.disabledTimer: Timer = Timer()
        self.autonomousCommand: Optional[Command] = None

        self.holding_alge = False

    @property
    def container(self) -> RobotContainer:
        return self._container

    # @tracer.start_as_current_span("robotInit")
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        if RobotBase.isSimulation():
            logger.setLevel(logging.INFO)
        else:
            logger.setLevel(logging.ERROR)

        logging.getLogger("wpilib").setLevel(logging.DEBUG)
        logging.getLogger("commands2").setLevel(logging.DEBUG)

        version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
        logger.info(f"Python: {version}, Software Version: {VERSION}")
        
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self._container = RobotContainer(self)
        self.disabledTimer = wpilib.Timer()

        # If this is a simulation, we need to silence joystick warnings
        if self.simulation:
            logger.warning("Simulation detected. Silencing annoying JoyStick warnings")
            DriverStation.silenceJoystickConnectionWarning(True)

    def robotPeriodic(self) -> None:
        """
        Periodic code for all modes should go here.

        This function is called each time a new packet is received from the driver
        station. Default period is 20 mS.
        """
        logger.info("*** called robotPeriodic")
        self._counter += 1

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run()

    def disabledInit(self) -> None:
        """
        Initialization code for disabled mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters disabled mode.
        """
        logger.info("*** called disabledInit")
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
        logger.info("*** called disabledPeriodic")

        if self.disabledTimer.hasElapsed(constants.WHEEL_LOCK_TIME):
            self.container.setMotorBrake(False)
            self.disabledTimer.stop()
            self.disabledTimer.reset()

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

        logger.info("*** called autonomousInit")
        self.container.setMotorBrake(True)

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
        logger.info("*** called autonomousPeriodic")

    def autonomousExit(self) -> None:
        """
        Exit code for autonomous mode should go here.

        Users should override this method for code which will be called each time
        the robot exits autonomous mode.
        """
        logger.info("*** autonomousExit: entry")

        command = self.container.get_autonomous_command()
        if command:
            command.cancel()

    def teleopInit(self) -> None:
        """
        Initialization code for teleop mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters teleop mode.
        """
        logger.info("*** called teleopInit")

        self.container.set_start_time()

        if self.autonomousCommand:
            self.autonomousCommand.cancel()
        else:
            CommandScheduler.getInstance().cancelAll()

    def teleopPeriodic(self) -> None:
        """
        Periodic code for teleop mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in teleop
        mode.
        """
        # Intake wheel control logic
        logger.info("*** called teleopPeriodic")

        # TODO: As we implement and understand the following, document what we are
        #       doing here.
        if self.holding_alge:
            self.intake_left.set(constants.D_ALGE_HOLD_SPEED)
            self.intake_right.set(-constants.D_ALGE_HOLD_SPEED)

        intake_coral = self.controller_shooter.getLeftBumperButton()
        if intake_coral:
            self.intake_left.set(constants.D_CORAL_INTAKE_SPEED)
            self.intake_right.set(-constants.D_CORAL_INTAKE_SPEED)
            self.holding_alge = False

        intake_alge = self.controller_shooter.getLeftTriggerAxis() >= 0.35
        if intake_alge:
            self.intake_left.set(constants.D_ALGE_INTAKE_SPEED)
            self.intake_right.set(-constants.D_ALGE_INTAKE_SPEED)
            self.holding_alge = True

        if b_shoot:
            self.intake_left.set(constants.D_SHOOT_SPEED)
            self.intake_right.set(-constants.D_SHOOT_SPEED)
            self.holding_alge = False

        # Elevator control logic
        pos_l0 = self.controller_shooter.getRightBumperButtonPressed()
        pos_l1 = self.controller_shooter.getAButtonPressed()
        pos_l2 = self.controller_shooter.getXButtonPressed()
        pos_l3 = self.controller_shooter.getYButtonPressed()

        if pos_l0:
            self.elevator.setControl(EL_POS_L0)

        if pos_l1:
            self.elevator.setControl(EL_POS_L1)

        if pos_l2:
            self.elevator.setControl(EL_POS_L2)

        if pos_l3:
            self.elevator.setControl(EL_POS_L3)

        pos_intake = self.controller_shooter.getBButtonPressed()
        if pos_intake:
            self.elevator.setControl(EL_POS_IN)

        intake_extend  = math.abs(controller_shooter.getLeftY())

        # Intake extender logic
        if intake_extend >= Constants.DEADBAND:
            self.intake_extend.setReference(intake_extend * constants.I_INTAKE_EXTEND_MAX,
                                            ControlType.kPosition)
        else:
            self.intake_extend.setReference(0, ControlType.kPosition)

        shoot     = self.controller_shooter.getRightTriggerAxis() >= 0.35
        alge_grab = self.controller_shooter.getRightStickButton()

        # Alge intake control logic
        if alge_grab:
            m_alge_roller.set(constants.D_ALGE_GRABBER_GRAB)
            mc_alge_rotation.setReference(constants.I_ALGE_ROTATION_OUT, ControlType.kPosition)

        elif shoot:
            m_alge_roller.set(constants.D_ALGE_GRABBER_SHOOT)
            mc_alge_rotation.setReference(constants.I_ALGE_ROTATION_IN, ControlType.kPosition)

        else:
            m_alge_roller.set(constants.D_ALGE_GRABBER_HOLD)
            mc_alge_rotation.setReference(constants.I_ALGE_ROTATION_IN, ControlType.kPosition)

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
