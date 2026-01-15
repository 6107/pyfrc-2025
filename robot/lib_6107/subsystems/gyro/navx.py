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

from wpilib import SmartDashboard, RobotBase, Timer
from wpilib import simulation
from wpimath.geometry import Rotation2d
from wpimath.units import degrees, degrees_per_second

try:
    import navx

    NAVX_SUPPORTED = True
except ImportError:
    NAVX_SUPPORTED = False

from lib_6107.constants import RADIANS_PER_DEGREE
from lib_6107.subsystems.gyro.gyro import Gyro, GyroIO


class NavX(Gyro):
    """
    NavX gyro implementation
    """
    gyro_type = "navX"

    def __init__(self, is_reversed: bool):
        super().__init__(is_reversed)
        self._gyro = navx.AHRS.create_spi()
        self._sim_gyro: Optional[Gyro] = None
        self._calibrated = False

    def initialize(self) -> None:
        """
        Perform initial steps to get your gyro ready
        """
        if self.is_calibrating:
            # Flag that gyro is not calibrated. Checked in periodic call
            self._calibrated = False
        else:
            self.zero_yaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
            self._calibrated = True

    @property
    def calibrated(self) -> bool:
        """
        Is this gyro calibrated. Implement in derived class if your gyro
        does not auto-calibrate.
        """
        return self._calibrated

    @property
    def is_calibrating(self) -> bool:
        """
        Is this gyro calibrated. Implement in derived class if your gyro
        does not auto-calibrate.
        """
        return self._gyro.isCalibrating()

    def reset(self, adjustment=None) -> None:
        """
        Reset the gyro
        """
        self._gyro.reset()

        # if adjustment is not None:
        #     # ADD adjustment - e.g trying to update the gyro from a pose
        #     self._gyro.setAngleAdjustment(adjustment)
        # else:
        #     # make sure there is no adjustment
        #     self._gyro.setAngleAdjustment(0)

    def zero_yaw(self) -> None:
        self._gyro.zeroYaw()

        if RobotBase.isSimulation():
            gyro = simulation.SimDeviceSim("navX-Sensor[4]")
            gyro_yaw = gyro.getDouble("Yaw")  # for some reason it seems we have to set Yaw and not Angle
            # gyro_angle = gyro.getDouble("Angle")
            # gyro_angle.set(0.0)
            gyro_yaw.set(0.0)

    @property
    def yaw(self) -> degrees:
        """
        helpful for determining nearest heading parallel to the wall,
        but you should probably never use this - just use get_angle to be consistent
        because yaw does NOT return the offset that get_Angle does
        """
        yaw = self._gyro.getYaw()

        return -yaw if self._reversed else yaw

    @property
    def pitch(self) -> degrees:
        pitch_offset = 0  # TODO: Always zero?

        return self._gyro.getPitch() - pitch_offset

    @property
    def roll(self) -> degrees:
        roll_offset = 0  # TODO: Always zero?

        return self._gyro.getRoll() - roll_offset

    @property
    def raw_angle(self) -> degrees:
        return self._gyro.getYaw()

    @property
    def angle(self) -> degrees:
        angle = self._gyro.getAngle()

        return -angle if self._reversed else angle

    @property
    def heading(self) -> Rotation2d:
        """
        Returns the heading of the robot
        """
        now = Timer.getFPGATimestamp()
        past = self._lastGyroAngleTime
        state = "ok"

        if not self._gyro.isConnected():
            state = "disconnected"
        else:
            notCalibrating = True
            if self._gyro.isCalibrating():
                notCalibrating = False
                state = "calibrating"

            raw_angle = self.raw_angle
            gyroAngle = -raw_angle if self._reversed else raw_angle

            # correct for gyro drift
            if self.gyroOvershootFraction != 0.0 and self._lastGyroAngle != 0 and notCalibrating:
                angleMove = gyroAngle - self._lastGyroAngle
                if abs(angleMove) > 15:  # if less than 10 degrees, adjust (otherwise it's some kind of glitch or reset)
                    print(f"WARNING: big angle move {angleMove} from {self._lastGyroAngle} to {gyroAngle}")
                else:
                    adjustment = -angleMove * self.gyroOvershootFraction
                    self._lastGyroAngleAdjustment += adjustment
                    self._gyro.setAngleAdjustment(max(-359, min(+359, self._lastGyroAngleAdjustment)))
                    # ^^ NavX code doesn't like angle adjustments outside (-360..+360) range

            self._lastGyroAngle = gyroAngle
            self._lastGyroAngleTime = now

        if state != self._lastGyroState:
            SmartDashboard.putString("gyro/state", f"{state} after {int(now - past)}s")
            self._lastGyroState = state

        last_angle = self._lastGyroAngle
        return Rotation2d.fromDegrees(last_angle)

    @property
    def turn_rate(self) -> float:
        """Returns the turn rate of the robot (in radians per second)

        :returns: The turn rate of the robot, in radians per second
        """
        return math.radians(self.turn_rate_degrees_per_second)

    @property
    def turn_rate_degrees_per_second(self) -> degrees_per_second:
        rate = self._gyro.getRate()

        return -rate if self._reversed else rate

    def periodic(self, inputs: GyroIO.GyroIOInputs) -> None:
        """
        Perform any periodic maintenance
        """
        if not self.calibrated and not self.is_calibrating:
            # Gyro has finished calibrating, set it to zero
            self.zero_yaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
            self._calibrated = True

    ######################
    # pykit / AdvantageScope support

    def updateInputs(self, inputs: GyroIO.GyroIOInputs) -> None:
        inputs.connected = self._gyro.isConnected()
        inputs.yawPosition = Rotation2d.fromDegrees(self.angle)

        gyroz = self._gyro.getRawGyroZ()
        if self.is_reversed:
            gyroz = -gyroz

        inputs.yawVelocityDegPerSec = gyroz * RADIANS_PER_DEGREE

    ######################
    # Simulation support

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Create your simulation gyro (and set  self._sim_gyro)
        """
        super().sim_init(physics_controller)

        # NavX (SPI interface)
        self._sim_gyro = simulation.SimDeviceSim("navX-Sensor[4]")

    @property
    def sim_yaw(self) -> degrees:
        return self._sim_gyro.getDouble("Yaw").get()

    @sim_yaw.setter
    def sim_yaw(self, value: degrees) -> None:
        """
        Used during simulation
        """
        # TODO: Copied from physics.py reconcile with any existing functions in this class
        if self._reversed:
            value = -value

        self._sim_gyro_state.set_raw_yaw(value)
