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
from typing import Optional

import sys
import wpilib
from commands2 import TimedCommandRobot, CommandScheduler
from commands2.command import Command
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
        station. All classes derived from 'Sybsystem' will have their 'periodic'
        function called automatically (right after this function). So only do
        non-Subsystem updates here

        Default period is 20 mS.
        """
        logger.debug(f"called robotPeriodic: enabled: {self.isEnabled()}")
        self._counter += 1

    def disabledInit(self) -> None:
        """
        Initialization code for disabled mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters disabled mode.
        """
        logger.debug("called disabledInit")
        self.container.disablePIDSubsystems()

        for subsystem in self.container.subsystems:
            if hasattr(subsystem, "stop") and callable(getattr(subsystem, "stop")):
                subsystem.stop()

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
        logger.debug("*** disabledExit: entry")
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
        logger.debug("*** called teleopInit")

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

    def teleopExit(self) -> None:
        """
        Exit code for teleop mode should go here.

        Users should override this method for code which will be called each time
        the robot exits teleop mode.
        """
        for subsystem in self.container.subsystems:
            if hasattr(subsystem, "stop") and callable(getattr(subsystem, "stop")):
                subsystem.stop()

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
