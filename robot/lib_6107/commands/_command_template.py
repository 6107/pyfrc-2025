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
#  This file originated from aesatchien's FRC2429_2025 project on github:
#       https://github.com/aesatchien/FRC2429_2025
#
#  It provides a starting place to define a Command. Just copy it to a new
#  filename and change the class name and implementation to suite what you
#  may need in your project.
#
from typing import Optional

from commands2 import Command
from wpilib import SmartDashboard


class CommandTemplate(Command):  # change the name for your command
    """
    TODO: Describe this class here
    """
    def __init__(self, container, indent: Optional[int]=0):
        super().__init__()
        self.setName("Sample Name")  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.start_time: float = 0
        self.print_end_message: bool = True

        # self.addRequirements(self.container.)  # commandsv2 version of requirements
        raise NotImplementedError("Remember to remove this line as well")

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time
        """
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'

        if self.print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
