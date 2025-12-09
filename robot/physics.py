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
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
from pyfrc.physics.core import PhysicsInterface
from wpimath.kinematics._kinematics import SwerveDrive4Kinematics, SwerveModulePosition
from wpimath.units import degrees

from frc_2025.reefscape import *
from frc_2025.subsystems.swervedrive.constants import DriveConstants
from robot import MyRobot

logger = logging.getLogger(__name__)


class PhysicsEngine:
    """
    Simulates a 2-wheel XRP robot using Arcade Drive joystick control.

    Any objects created or manipulated in this file are for simulation purposes only.
    """
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        logger.info("PhysicsEngine: entry")

        self._physics_controller = physics_controller
        self._robot: MyRobot = robot
        self._drivetrain = robot.container.robotDrive

        # Initialize our simulated subsystems
        self._initialize_swerve()

        # TODO: Bunch of work needed here?  Perhaps setting our pose as least

        # Set up field, it is declared in the physics controller simulation file
        # and initialized in the _simulationInit() method and it initializes teh
        # SmartDashboard.
        self.field = physics_controller.field

        # Register for any changes in alliance before the match starts
        robot.container.register_alliance_change_callback(self._alliance_change)
        self._alliance_change(self._robot.container.is_red_alliance)

        # TODO: If vision odometry is supported in simulation, this may need to be
        #       changed to the robot's field view and not the 'overhead' view of the
        #       playing field.

    @property
    def yaw(self) -> degrees:
        return self._gyro_yaw.get()

    @yaw.setter
    def yaw(self, value: degrees) -> None:
        self._gyro_yaw.set(value)

    def update_sim(self, _now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param _now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        if self._robot.isEnabled():
            logger.debug(f"PhysicsEngine: updating sim: Enabled: {self._robot.isEnabled()}")

            # TODO: Add update sim methods for other subsystems (return amps used)
            self._update_swerve(tm_diff)  # TODO: Return amps used
            # TODO: update battery with amps consumed

    def _alliance_change(self, is_red: bool) -> None:
        """
        Called whenever the alliance changes colors before the match / competition begins
        """
        initial_pose = RED_TEST_POSE if is_red else BLUE_TEST_POSE
        self._physics_controller.field.setRobotPose(initial_pose)

    def _initialize_swerve(self):
        self.kinematics: SwerveDrive4Kinematics = DriveConstants.kDriveKinematics  # our swerve drive kinematics

        # NavX (SPI interface)
        self._gyro = simlib.SimDeviceSim("navX-Sensor[4]")
        self._gyro_yaw = self._gyro.getDouble("Yaw")  # for some reason it seems we have to set Yaw and not Angle

        # kinematics chassis speeds wants them in same order as in original definition - unfortunate ordering
        spark_drives = ['lf_drive', 'rf_drive', 'lb_drive', 'rb_drive']
        spark_drive_ids = [21, 25, 23, 27]  # keep in this order - based on our kinematics definition
        self.spark_turns = ['lf_turn', 'rf_turn', 'lb_turn', 'rb_turn']
        spark_turn_ids = [20, 24, 22, 26]  # keep in this order

        # Got rid of last year's elements: 'br_crank', 'bl_crank', 'tr_crank', 'tl_crank', 't_shooter', 'b_shooter'
        spark_peripherals = ['intake', 'indexer']
        spark_peripheral_ids = [5, 12]  # Kept  'indexer' id as 12 because it came last before removing the elements

        # allow ourselves to access the sim device's Position, Velocity, Applied Output, etc
        spark_names = spark_drives + self.spark_turns + spark_peripherals
        spark_ids = spark_drive_ids + spark_turn_ids + spark_peripheral_ids

        # create a dictionary so we can refer to the sparks by name and get their relevant parameters
        self.spark_dict = {}
        for idx, (spark_name, can_id) in enumerate(zip(spark_names, spark_ids)):
            spark = simlib.SimDeviceSim(f'SPARK MAX [{can_id}]')

            self.spark_dict[spark_name] = {
                'controller': spark,
                'position': spark.getDouble('Position'),
                'velocity': spark.getDouble('Velocity'),
                'output': spark.getDouble('Applied Output')
            }
        # for key, value in self.spark_dict.items():  # see if these make sense
        #     print(f'{key}: {value}')

        # self.distances = [0, 0, 0, 0]

        # set up the initial location of the robot on the field
        self._alliance_change(self._robot.container.is_red_alliance)

    def _update_swerve(self, tm_diff):
        log_it = self._robot.counter % 20 == 0

        if log_it:
            logger.debug("Update swerve:===========================================")
            logger.debug(f"Update swerve: Entry. tm_diff: {tm_diff:.4f}")

        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        target_angles = [wpilib.SmartDashboard.getNumberArray(dash_value, [0, 0])[1] for dash_value in dash_values]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)  # this works to update the simulated spark

        if self._robot.counter % 10 == 0 and self._robot.isEnabled():
            wpilib.SmartDashboard.putNumberArray('target_angles', target_angles)

        # send the speeds and positions from the spark sim devices to the fourmotorswervedrivetrain
        # module_states = [SwerveModuleState(self.spark_dict[drive]['velocity'].value,
        #                                    geo.Rotation2d(self.spark_dict[turn]['position'].value))
        #                  for drive, turn in zip(spark_drives, self.spark_turns)]

        # using our own kinematics to update the chassis speeds
        module_states = self._drivetrain.get_desired_swerve_module_states()
        speeds = self.kinematics.toChassisSpeeds(module_states)

        if log_it:
            logger.debug(f"Update swerve before drive command: module states: {module_states}, speeds: {speeds}")

        # update the sim's robot. Returned value is same as what is returned from self._physics_controller.get_pose()
        pose = self._physics_controller.drive(speeds, tm_diff)

        # Limit it to the field size (manually)  TODO: Is there a programmatic way to do this?
        x = min(17.5, max(0.0, pose.x))
        y = min(8.0, max(0.0, pose.y))
        new_pose = Pose2d(x, y, pose.rotation())

        self._drivetrain.resetSimPose(new_pose, [SwerveModulePosition()] * 4,
                                      self._physics_controller.get_pose().rotation())
        previous = self.yaw
        omega = speeds.omega
        gyro_degrees = math.degrees(speeds.omega * tm_diff)
        new = previous - math.degrees(speeds.omega * tm_diff)

        if log_it:
            logger.debug(f"Update swerve: previous: {previous}, new: {new}, omega: {omega}, degrees: {gyro_degrees}")

        gyro_degrees = pose.rotation().degrees()
        self.yaw = -gyro_degrees if DriveConstants.kGyroReversed else gyro_degrees
