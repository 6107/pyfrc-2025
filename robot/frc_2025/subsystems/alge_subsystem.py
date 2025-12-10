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
from rev import SparkMax, SparkFlex, SparkLowLevel, SparkClosedLoopController
from wpilib import SmartDashboard

# from frc_2025.subsystems.constants import D_ALGE_HOLD_SPEED, D_CORAL_INTAKE_SPEED, D_ALGE_INTAKE_SPEED, \
#     D_SHOOT_SPEED, D_ALGE_GRABBER_GRAB, D_ALGE_GRABBER_SHOOT, D_ALGE_GRABBER_HOLD, I_ALGE_ROTATION_OUT, \
#     I_ALGE_ROTATION_IN

logger = logging.getLogger(__name__)


class AlgeRoller:
    def __init__(self, device_id: int, container: 'RobotContainer') -> None:
        self._rcontainer = container

        self._motor = SparkFlex(device_id, SparkLowLevel.MotorType.kBrushless)

        # motorConfig = SparkBaseConfig()     # TODO: Needed?  What else can it do for us
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
    def speed(self) -> float:
        return self._speed

    @speed.setter
    def speed(self, value: float) -> None:
        logger.info(f"AlgeRoller: speed: {value:.1f}")

        # TODO: Should we have a max/min check here?

        self._motor.set(value)
        self._speed = value
        SmartDashboard.putNumber("Alge Roller Speed", value)

    def stop(self) -> None:
        logger.info("AlgeRoller: stopped")
        self.speed = 0.0

    def periodic(self) -> None:
        pass  # TODO: Anything here ?

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


class AlgeRotation:
    def __init__(self, device_id: int, container: 'RobotContainer') -> None:
        self._rcontainer = container

        self._motor = SparkMax(device_id, SparkLowLevel.MotorType.kBrushless)

        # motorConfig = SparkBaseConfig()     # TODO: Needed?  What else can it do for us
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
    def closed_loop_controller(self) -> SparkClosedLoopController:
        return self._motor.getClosedLoopController()

    @property
    def speed(self) -> float:
        return self._speed

    @speed.setter
    def speed(self, value: float) -> None:
        logger.info(f"AlgeRotation: speed: {value:.1f}")

        # TODO: Should we have a max/min check here?

        self._motor.set(value)
        self._speed = value
        SmartDashboard.putNumber("Alge Rotation Speed", value)

    def stop(self) -> None:
        logger.info("AlgeRotation: stopped")
        self.speed = 0.0

    def periodic(self) -> None:
        pass  # TODO: Anything here ?

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


class AlgeSubsystem(Subsystem):
    """
    The Alge subsystem consists of a roller and rotation motor. Those are not
    implemented as independent subsystems as they need to work in unison.
    """

    def __init__(self, roller_device_id: int, rotation_device_id: int, container: 'RobotContainer') -> None:
        super().__init__()

        self._container = container
        self._roller = AlgeRoller(roller_device_id, container)
        self._rotation = AlgeRotation(rotation_device_id, container)

        self.holding_alge = False
        self.shoot = False

    def stop(self) -> None:
        self._roller.speed = 0.0
        self._rotation.speed = 0.0

    @property
    def alge_roller(self) -> AlgeRoller:
        return self._roller

    @property
    def alge_rotation(self) -> AlgeRotation:
        return self._rotation

    def configure_button_bindings(self, driver, shooter) -> None:
        """
        Configure the driver and shooter joystick controls here
        """
        pass  # TODO: Add me

    def initialize_dashboard(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        pass  # TODO: Add me

    def periodic(self) -> None:
        pass
        # TODO: Can following be done with commands
        shooter = self._container.controller_shooter

        intake_coral = shooter.getLeftBumperButton()
        intake_alge = shooter.getLeftTriggerAxis() >= 0.35
        self.shoot = shooter.getRightTriggerAxis() >= 0.35

        if intake_coral:
            self.holding_alge = None

        if intake_alge:
            self.holding_alge = True
        #
        # alge_grab = shooter.getRightStickButton()
        #
        # # Alge intake control logic
        # if alge_grab:
        #     self.alge_roller.speed = D_ALGE_GRABBER_GRAB
        #     self.alge_rotation.closed_loop_controller.setReference(I_ALGE_ROTATION_OUT,
        #                                                            SparkBase.ControlType.kPosition)
        #
        # elif self.shoot:
        #     self.alge_roller.speed = D_ALGE_GRABBER_SHOOT
        #     self.alge_rotation.closed_loop_controller.setReference(I_ALGE_ROTATION_IN,
        #                                                            SparkBase.ControlType.kPosition)
        #
        # else:
        #     self.alge_roller.speed = D_ALGE_GRABBER_HOLD
        #     self.alge_rotation.closed_loop_controller.setReference(I_ALGE_ROTATION_IN,
        #                                                            SparkBase.ControlType.kPosition)

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
