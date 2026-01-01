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

from commands2 import Subsystem
from phoenix6.controls.position_voltage import PositionVoltage
from phoenix6.hardware.talon_fx import TalonFX
from wpilib import SmartDashboard

from frc_2025.subsystems.constants import EL_POS_L0, EL_POS_L1, \
    EL_POS_L2, EL_POS_L3, EL_POS_IN

SUPPORTED_POSITIONS = (EL_POS_L0, EL_POS_L1,
                       EL_POS_L2, EL_POS_L3,
                       EL_POS_IN)

logger = logging.getLogger(__name__)


class Elevator(Subsystem):
    def __init__(self, can_bus_device_id: int, container: 'RobotContainer') -> None:
        super().__init__()

        self._robot = container.robot

        self._motor: TalonFX = TalonFX(can_bus_device_id)
        self._position: PositionVoltage = EL_POS_L0

    def stop(self) -> None:
        self.position = 0.0
        logger.info("Elevator: stopped")

    @property
    def position(self) -> PositionVoltage:
        return self._position

    @position.setter
    def position(self, value: PositionVoltage) -> None:
        if value not in SUPPORTED_POSITIONS:
            logger.error(f"Elevator: Position value not supported: {value}")
            return

        logger.info(f"Elevator: Position: {value}")
        self._motor.set_control(value)
        self._position = value

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        pass

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        divisor = 10 if self._robot.isEnabled() else 20
        update_dash = self._robot.counter % divisor == 0

        if update_dash:
            SmartDashboard.putNumber("Elevator/Position", self._position.position)
            SmartDashboard.putNumber("Elevator/Velocity", self._position.velocity)

    def configure_button_bindings(self, driver, shooter) -> None:
        """
        Configure the driver and shooter joystick controls here
        """
        pass  # TODO: Add me

    def periodic(self) -> None:
        pass  # TODO: Anything here ?

        # # Elevator control logic
        # shooter = self._container.controller_shooter
        #
        # pos_l0 = shooter.getRightBumperButtonPressed()
        # pos_l1 = shooter.getAButtonPressed()
        # pos_l2 = shooter.getXButtonPressed()
        # pos_l3 = shooter.getYButtonPressed()
        # pos_intake = shooter.getBButtonPressed()
        #
        # # TODO: Need to do if-elif-else logic here or turn these into
        # #       commands
        # if pos_l0:
        #     self.position = EL_POS_L0
        #
        # if pos_l1:
        #     self.position = EL_POS_L1
        #
        # if pos_l2:
        #     self.position = EL_POS_L2
        #
        # if pos_l3:
        #     self.position = EL_POS_L3
        #
        # if pos_intake:
        #     self.position = EL_POS_IN

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """

    def simulationPeriodic(self, **kwargs) -> Optional[float]:
        """
        This method is called periodically by the CommandScheduler (after the periodic
        function. It is useful for updating subsystem-specific state that needs to be
        maintained for simulations, such as for updating simulation classes and setting
        simulated sensor readings.

        Unlike the physics 'update_sim', it is not called with the current time (now)
        or the amount of time since 'update_sim' was called (tm_diff).  It is called
        just after the 'periodic' call and before the 'update_sim' is called.

        To unify the two uses, our call signature above has a kwargs parameter so we
        know when we are being called. Typically, we only need to support one method
        but for future simulation purposes, if called with keywords, return the amperage
        used in this interval
        """
        # For the swerve drive, we only support the 'update_sim' form of call
        if not kwargs:
            return None

        amperes_used = 0.0  # TODO: Support in future

        # TODO: Anything here

        return amperes_used
