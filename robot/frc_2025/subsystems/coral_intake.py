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
from rev import SparkMax, SparkLowLevel, SparkBaseConfig, SparkClosedLoopController
from wpilib import SmartDashboard

logger = logging.getLogger(__name__)


class CoralIntake(Subsystem):
    def __init__(self, can_bus_device_id: int, container: 'RobotContainer', name: str, inverted=True) -> None:
        super().__init__()

        self._container = container
        self._name = name
        self._motor = SparkMax(can_bus_device_id, SparkLowLevel.MotorType.kBrushless)
        self._alge_subsystem = container.alge_subsystem

        motorConfig = SparkBaseConfig()  # TODO: Needed?  What else can it do for us
        # motorConfig.inverted(inverted)
        # motorConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
        # motorConfig.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen)
        # motorConfig.limitSwitch.forwardLimitSwitchEnabled(True)  # by default disabled (until `intakeGamepiece()`)
        # self._motor.configure(motorConfig,
        #                       SparkBase.ResetMode.kResetSafeParameters,
        #                       SparkBase.PersistMode.kPersistParameters)

        # initial state
        self._speed = 0.0

    @property
    def closed_loop_controller(self) -> Optional[SparkClosedLoopController]:
        return None

    @property
    def speed(self) -> float:
        return self._speed

    @speed.setter
    def speed(self, value: float) -> None:
        logger.info(f"CoralIntake: {self._name} speed: {value:.1f}")

        # TODO: Should we have a max/min check here?

        self._motor.set(value)
        self._speed = value
        SmartDashboard.putNumber(f"{self._name} Speed", value)

    def stop(self) -> None:
        self.speed = 0.0
        logger.info("CoralIntake: stopped")

    def initialize_dashboard(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        pass  # TODO: Add me

    def configure_button_bindings(self, driver, shooter) -> None:
        """
        Configure the driver and shooter joystick controls here
        """
        pass  # TODO: Add me

    def periodic(self) -> None:
        pass

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        pass

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


class LeftCoralIntake(CoralIntake):
    def __init__(self, can_bus_device_id: int, robot: 'MyRobot', inverted=True) -> None:
        super().__init__(can_bus_device_id, robot, "Left Intake", inverted=inverted)

    def periodic(self) -> None:
        pass  # TODO: Can we use commands?
        #
        # if self._alge_subsystem.holding_alge is True:
        #     self.speed = D_ALGE_HOLD_SPEED
        #
        # elif self._alge_subsystem.holding_alge is False:
        #     self.speed = D_ALGE_INTAKE_SPEED
        #
        # elif self._alge_subsystem.holding_alge is None:
        #     self.speed = D_CORAL_INTAKE_SPEED
        #
        # elif self._alge_subsystem.shoot:
        #     self.speed = D_SHOOT_SPEED


class RightCoralIntake(CoralIntake):
    def __init__(self, can_bus_device_id: int, robot: 'MyRobot', inverted=True) -> None:
        super().__init__(can_bus_device_id, robot, "Right Intake", inverted=inverted)

    def periodic(self) -> None:
        pass  # TODO: Can we use commands?
        #
        # if self._alge_subsystem.holding_alge is True:
        #     self.speed = -D_ALGE_HOLD_SPEED
        #
        # elif self._alge_subsystem.holding_alge is False:
        #     self.speed = -D_ALGE_INTAKE_SPEED
        #
        # elif self._alge_subsystem.holding_alge is None:
        #     self.speed = -D_CORAL_INTAKE_SPEED
        #
        # elif self._alge_subsystem.shoot:
        #     self.speed = -D_SHOOT_SPEED


class ExtendCoralIntake(CoralIntake):
    def __init__(self, can_bus_device_id: int, robot: 'MyRobot', inverted=True) -> None:
        super().__init__(can_bus_device_id, robot, "Intake Extend", inverted=inverted)

    @property
    def closed_loop_controller(self) -> Optional[SparkClosedLoopController]:
        return self._motor.getClosedLoopController()

    def periodic(self) -> None:
        if self._alge_subsystem.shoot:
            self._alge_subsystem.shoot = False
