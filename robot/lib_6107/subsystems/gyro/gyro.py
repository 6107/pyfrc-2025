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
import math
from typing import Optional

from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import degrees, degrees_per_second

from lib_6107.subsystems.pykit.gyro_io import GyroIO


class Gyro(GyroIO):
    """
    Gyro is the base class for gyros on our system. Actual gyros are derived
    from this class.
    """
    gyro_type = "unknown"

    def __init__(self, is_reversed: bool) -> None:
        super().__init__()

        self._reversed = is_reversed
        self._sim_gyro: Optional[Gyro] = None
        self._physics_controller: Optional['PhysicsInterface'] = None

    def initialize(self) -> None:
        """
        Perform initial steps to get your gyro ready
        """
        self.reset()

    @property
    def is_reversed(self) -> bool:
        return self._reversed

    @property
    def calibrated(self) -> bool:
        """
        Is this gyro calibrated. Implement in derived class if your gyro
        does not auto-calibrate.
        """
        return True

    @property
    def is_calibrating(self) -> bool:
        """
        Is this gyro calibrated. Implement in derived class if your gyro
        does not auto-calibrate.
        """
        return False

    def reset(self) -> None:
        """
        Reset the gyro
        """
        raise NotImplemented("Implement in derived class")

    def zero_yaw(self) -> None:
        raise NotImplemented("Implement in derived class")

    @property
    def yaw(self) -> degrees:
        """
        helpful for determining nearest heading parallel to the wall,
        but you should probably never use this - just use get_angle to be consistent
        because yaw does NOT return the offset that get_Angle does
        """
        raise NotImplemented("Implement in derived class")

    @property
    def pitch(self) -> degrees:
        raise NotImplemented("Implement in derived class")

    @property
    def roll(self) -> degrees:
        raise NotImplemented("Implement in derived class")

    @property
    def angle(self) -> degrees:
        raise NotImplemented("Implement in derived class")

    @property
    def raw_angle(self) -> degrees:
        raise NotImplemented("Implement in derived class")

    @property
    def heading(self) -> Rotation2d:
        """
        Returns the heading of the robot
        """
        return Rotation2d.fromDegrees(self.yaw)

    @property
    def turn_rate(self) -> float:
        """Returns the turn rate of the robot (in radians per second)

        :returns: The turn rate of the robot, in radians per second
        """
        return math.radians(self.turn_rate_degrees_per_second)

    @property
    def turn_rate_degrees_per_second(self) -> degrees_per_second:
        raise NotImplemented("Implement in derived class")

    def periodic(self, inputs: GyroIO.GyroIOInputs) -> None:
        """
        Perform any periodic maintenance
        """
        self.updateInputs(inputs)

    ######################
    # SmartDashboard support

    def dashboard_initialize(self) -> None:
        SmartDashboard.putString('Gyro/type', self.gyro_type)

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        SmartDashboard.putNumber('Gyro/angle', self.angle)
        SmartDashboard.putNumber('Gyro/yaw', self.yaw)
        SmartDashboard.putNumber('Gyro/pitch', self.pitch)
        SmartDashboard.putNumber('Gyro/roll', self.roll)

    ######################
    # Simulation support

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Create your simulation gyro (and set  self._sim_gyro)
        """

    @property
    def sim_yaw(self) -> degrees:
        raise NotImplemented("Implement in derived class")

    @sim_yaw.setter
    def sim_yaw(self, value: degrees) -> None:
        """
        Used during simulation
        """
        raise NotImplemented("Implement in derived class")
