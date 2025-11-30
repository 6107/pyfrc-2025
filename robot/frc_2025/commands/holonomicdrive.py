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
# From Gene Panov's (Team 714) CommandRevSwerve project (and FRC Python videos)


import logging

import commands2
from wpimath import applyDeadband

logger = logging.getLogger(__name__)


class HolonomicDrive(commands2.Command):
    """
    "holonomic" means that it can rotate independently of driving forward or left
    (examples: mecanum drivetrain, ball drivetrain, swerve drivetrain)
    """

    def __init__(self, robot_container, drivetrain, forwardSpeed, leftSpeed, rotationSpeed, deadband=0, **kwargs):
        """
        Drive the robot at `driveSpeed` and `rotationSpeed` until this command is terminated.
        """
        super().__init__()

        self.robot = robot_container.robot

        self.forwardSpeed = forwardSpeed
        if not callable(forwardSpeed):
            self.forwardSpeed = lambda: forwardSpeed

        self.leftSpeed = leftSpeed
        if not callable(leftSpeed):
            self.leftSpeed = lambda: leftSpeed

        self.rotationSpeed = rotationSpeed
        if not callable(rotationSpeed):
            self.rotationSpeed = lambda: rotationSpeed

        assert deadband >= 0, f"deadband={deadband} is not positive"
        self.deadband = deadband

        self.drivetrain = drivetrain
        self.kwargs = kwargs

        self.addRequirements(drivetrain)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        if self.robot.isEnabled() and self.robot.counter % 20 == 0:
            forwardSpeed = self.forwardSpeed()
            leftSpeed = self.leftSpeed()
            rotationSpeed = self.rotationSpeed()

            logger.debug(
                f"HolonomicDrive command (before): forward={forwardSpeed}, left={leftSpeed}, rotation={rotationSpeed}, deadband={self.deadband}")
            logger.debug(
                f"HolonomicDrive command (before): forward={applyDeadband(self.forwardSpeed(), self.deadband)}, left={applyDeadband(self.leftSpeed(), self.deadband)}, rotation={applyDeadband(self.rotationSpeed(), self.deadband)}")

        self.drivetrain.drive(
            applyDeadband(self.forwardSpeed(), self.deadband),
            applyDeadband(self.leftSpeed(), self.deadband),
            applyDeadband(self.rotationSpeed(), self.deadband),
            **self.kwargs
        )

    def end(self, interrupted: bool):
        self.drivetrain.stop()  # stop immediately if command is ending
