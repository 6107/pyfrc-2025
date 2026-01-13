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

import collections
from typing import ClassVar

from commands2 import CommandScheduler
from wpimath.units import seconds

from pykit.loggedrobot import LoggedRobot


class TimedCommandLoggedRobot(LoggedRobot):
    """
    A pyrobot 'TimedCommandRobot' merged with a pykit 'LoggedRobot'

    The wpilib 'TimedCommandRobot' extends 'TimedRobot' and:
      - adds a periodic call to the CommandScheduler at the initialization of
        the robot.

        'TimedRobot' extends 'IterativeRobotBase' and:
           - defines kDefaultPeriod (float) of 20.0, which does not appear to
             be used anywhere but the initializer. It does have an __init__
             keyword parameter named 'period' that is set to 0.02 seconds.

           - Adds the following funtions. Note that these are just functions
             with no implementation to allow derived classed to add their own
             functionality here:
              - addPeriodic(self, callback: collections.abc.Callable[[], None],
                            period: wpimath.units.seconds,
                            offset: wpimath.units.seconds = 0.0) -> None:
              - endCompetition(self) -> None
              - getLoopStartTime(self) -> int
              - startCompetition(self) -> None

    The LoggedRobot from pykit (Team 1757) extends "IterativeRobotBase" and:
      - integrates with the `Logger` to automatically handle the logging of
        robot data and periodic loops.

        This is done by defining a 'default_period' of 0.02 (which is different
        attribute name from 'TimedRobots' kDefaultPeriod). See the not on the
        'period' keyword argument in 'TimedRobot' that matches the class attribute
        and initializer that 'LoggedRobot' uses.

      - LoggedRobot also extends 'IterativeRobotBase' and adds the following functions:
         - endCompetition(self) -> None
         - startCompetition(self) -> None
    """
    #
    kDefaultPeriod: ClassVar[float] = 20.0
    kSchedulerOffset = 0.005
    default_period = 0.02  # seconds

    def __init__(self, period: seconds = default_period) -> None:
        self.default_period = period

        # Call into pykit's base class
        super().__init__()

        # Add the 'TimeCommandRobot' call to initialize the command scheduler
        self.addPeriodic(CommandScheduler.getInstance().run, period, self.kSchedulerOffset)

    # Add the missing 'TimedRobot' functions that you can override in your robot

    def addPeriodic(self, callback: collections.abc.Callable[[], None], period: seconds,
                    offset: seconds = 0.0) -> None:
        """
        Add a callback to run at a specific period with a starting time offset.

        This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback
        run synchronously. Interactions between them are thread-safe.

        :param callback: The callback to run.
        :param period:   The period at which to run the callback.
        :param offset:   The offset from the common starting time. This is useful
                         for scheduling a callback in a different timeslot relative
                         to TimedRobot.
        """

    def getLoopStartTime(self) -> int:
        """
        Return the system clock time in microseconds for the start of the current
        periodic loop. This is in the same time base as Timer.GetFPGATimestamp(),
        but is stable through a loop. It is updated at the beginning of every
        periodic callback (including the normal periodic loop).

        :returns: Robot running time in microseconds, as of the start of the current
                  periodic function.
        """
