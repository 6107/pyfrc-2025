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
import math
from typing import Callable, Tuple, Optional, List, Dict, Union, Any

import navx
import ntcore
import robotpy_apriltag as apriltag
from commands2 import Subsystem, TimedCommandRobot, InstantCommand
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.config import RobotConfig
from phoenix6.hardware import pigeon2
from rev import SparkMax
from wpilib import SmartDashboard, Field2d, RobotBase, Timer, DriverStation
from wpilib import simulation
from wpimath.controller import PIDController
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Transform2d, Transform3d, Translation3d, Rotation3d, \
    Pose3d, Rotation2d, Translation2d, Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics, \
    SwerveDrive4Odometry, SwerveModulePosition
from wpimath.units import degrees, degrees_per_second, inchesToMeters

from frc_2025.reefscape import RED_TEST_POSE, BLUE_TEST_POSE
from frc_2025.subsystems import constants
from frc_2025.subsystems.constants import DeviceID
from frc_2025.subsystems.swervedrive import swerveutils
from frc_2025.subsystems.swervedrive.constants import DriveConstants
from frc_2025.subsystems.swervedrive.maxswervemodule import MAXSwerveModule

# TODO: This value needs to be tested. Perform the following on a real robot
#
# Measuring Overshoot
# Implement a Control Loop:
#   You cannot simply apply constant power until the target angle is reached, as the robot needs time to decelerate
#   and will inevitably overshoot. A simple Proportional (P) loop is the standard starting point for FRC teams.
#   The motor power is made proportional to the difference between the target angle and the current angle.
#
#      Formula (simplified P-loop): motorPower = (targetAngle - currentAngle) * kP
#      kP is a constant you tune to get the desired performance.
#
# Log Data:
#   Use your FRC development environment (e.g., WPILib) to log the robot's current gyro angle and the target
#   angle to a file or SmartDashboard/Shuffleboard.
#
# Perform a Test Turn:
#   Command your robot to turn to a specific, significant angle (e.g., 90 degrees) using the P-loop,
#   and log the data during the process.
#
# Analyze the Data:
#   After the test, view the logged data in a graph or spreadsheet.
# Target Angle:
#   The desired final angle (e.g., 90 degrees).
#
# Peak Angle:
#   The maximum angle the robot reaches during the turn before it starts correcting back towards the target.
#
# Calculate Overshoot:
#   The difference between the peak angle and the target angle is the overshoot.
#
# Overshoot = Peak Angle - Target Angle
#
# Correcting Overshoot
#
# The primary method for reducing overshoot is tuning your control loop.
#
# Adjust kP:
#   If your robot consistently overshoots significantly, your kP value is likely too high. Lowering it
#   will make the turn slower but more accurate.
#
# Add Derivative (D) control:
#   Implementing a full PID loop can help. The derivative term (kD) dampens the system by applying a
#   counter-force based on how fast the error is changing (i.e., the robot's turn rate), which helps
#   slow the robot down as it approaches the target.
#
# Slow Down Turns:
#   As a simple fix, reducing the maximum motor power or speed used for turns will also reduce overshoot,
#   though it makes the robot slower overall.
#
# Ensure Proper Calibration:
#   Make sure the gyro is stationary during its initial calibration phase (when the robot code starts)
#   to minimize drift and baseline errors.
#
GYRO_OVERSHOOT_FRACTION = -3.25 / 360
# ^^ our gyro didn't overshoot, it "undershot" by 0.1 degrees in a 360 degree turn

SwerveModuleStats = Tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]

logger = logging.getLogger(__name__)


class DriveSubsystem(Subsystem):
    def __init__(self, container: 'RobotContainer',
                 maxSpeedScaleFactor: Optional[Callable[[None], float]] = None,
                 **kwargs: Optional[Dict[str, Any]]) -> None:
        super().__init__()
        if maxSpeedScaleFactor is not None:
            assert callable(maxSpeedScaleFactor)

        self._container = container
        self._robot = container.robot

        # Camera/localizer defaults
        self.front_camera = None
        self.vision_odometry = False
        self.field_relative = False

        cameras: Dict[str, Any] = kwargs.get("Cameras")
        if cameras is not None and "Front" in cameras:
            self.front_camera = cameras["Front"]["Camera"]

            localizer = cameras["Front"].get("Localizer")
            if localizer:
                self.vision_odometry = True
                self.field_relative = True
                self.localizer = localizer

        self.maxSpeedScaleFactor: Optional[Callable[[None], float]] = maxSpeedScaleFactor

        self.gyroOvershootFraction = 0.0
        if not TimedCommandRobot.isSimulation():
            self.gyroOvershootFraction = GYRO_OVERSHOOT_FRACTION

        enabledChassisAngularOffset = 0 if DriveConstants.kAssumeZeroOffsets else 1

        # Create MAXSwerveModules
        self.frontLeft = MAXSwerveModule(
            DeviceID.DRIVETRAIN_LEFT_FRONT_DRIVING_ID,
            DeviceID.DRIVETRAIN_LEFT_FRONT_TURNING_ID,
            DriveConstants.kFrontLeftAngularOffset,
            driveMotorInverted=DriveConstants.kFrontLeftDriveMotorInverted,
            turnMotorInverted=DriveConstants.kFrontLeftTurningMotorInverted,
            motorControllerType=SparkMax,
            cancoder_device_id=DeviceID.DRIVETRAIN_LEFT_FRONT_ENCODER_ID,
            label="lf"
        )

        self.frontRight = MAXSwerveModule(
            DeviceID.DRIVETRAIN_RIGHT_FRONT_DRIVING_ID,
            DeviceID.DRIVETRAIN_RIGHT_FRONT_TURNING_ID,
            DriveConstants.kFrontRightChassisAngularOffset * enabledChassisAngularOffset,
            driveMotorInverted=DriveConstants.kFrontRightDriveMotorInverted,
            turnMotorInverted=DriveConstants.kFrontRightTurningMotorInverted,
            motorControllerType=SparkMax,
            cancoder_device_id=DeviceID.DRIVETRAIN_RIGHT_FRONT_ENCODER_ID,
            label="rf"
        )

        self.rearLeft = MAXSwerveModule(
            DeviceID.DRIVETRAIN_LEFT_REAR_DRIVING_ID,
            DeviceID.DRIVETRAIN_LEFT_REAR_TURNING_ID,
            DriveConstants.kRearLeftAngularOffset,
            driveMotorInverted=DriveConstants.kRearLeftDriveMotorInverted,
            turnMotorInverted=DriveConstants.kRearLeftTurningMotorInverted,
            motorControllerType=SparkMax,
            cancoder_device_id=DeviceID.DRIVETRAIN_LEFT_REAR_ENCODER_ID,
            label="lb"
        )

        self.rearRight = MAXSwerveModule(
            DeviceID.DRIVETRAIN_RIGHT_REAR_DRIVING_ID,
            DeviceID.DRIVETRAIN_RIGHT_REAR_TURNING_ID,
            DriveConstants.kRearRightAngularOffset,
            driveMotorInverted=DriveConstants.kRearRightDriveMotorInverted,
            turnMotorInverted=DriveConstants.kRearRightTurningMotorInverted,
            motorControllerType=SparkMax,
            cancoder_device_id=DeviceID.DRIVETRAIN_RIGHT_REAR_ENCODER_ID,
            label="rb"
        )
        self.swerve_modules: List[MAXSwerveModule] = [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]

        # TODO: Currently only the Phoenix6 Pigeon2 is supported.  Some work had been done with the NavX
        #       encoder, but it was not completed or tested outside of simulations/physics.  In this
        #       initial implementation, I am using the 2025 Java YAGSL code with what the CyberJagzz
        #       robot Java based code used.
        #
        # TODO: In the future, look at what the YAGSL 'SwerveIMU' class provides and create one of our
        #       own to track it.
        # The gyro sensor
        self._gyro: Optional[Union[Any, pigeon2.Pigeon2]] = None

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            self._gyro = navx.AHRS.create_spi()
            if self._gyro.isCalibrating():
                # Flag that gyro is not calibrated. Checked in periodic call
                self.gyro_calibrated = False
            else:
                self.zero_yaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
                self.gyro_calibrated = True

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            # Note: Default pigeon2 config has compass disabled. We want it that way as well.
            self._gyro: pigeon2.Pigeon2 = pigeon2.Pigeon2(DriveConstants.kGyroDeviceId)
            self._gyro.reset()

        # timer and variables for checking if we should be using pid on rotation
        self.keep_angle = 0.0  # the heading we try to maintain when not rotating
        self.keep_angle_timer = Timer()
        self.keep_angle_timer.start()
        self.keep_angle_timer.reset()
        self.keep_angle_pid = PIDController(0.015, 0, 0)  # TODO: put these in constants.  allow 1% stick per degree
        self.keep_angle_pid.enableContinuousInput(-180, 180)  # using the gyro's yaw is b/w -180 and 180
        self.last_rotation_time = 0
        self.time_since_rotation = 0
        self.last_drive_time = 0
        self.time_since_drive = 0

        self.fwd_magLimiter = SlewRateLimiter(0.9 * DriveConstants.kMagnitudeSlewRate)
        self.strafe_magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)

        # TODO: original gyro attributes below from the Java 2025 code.
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0
        self._lastGyroAngleAdjustment = 0
        self._lastGyroState = "ok"

        # Slew rate filter variables for controlling lateral acceleration
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0
        self.xSpeedDelivered = 0.0
        self.ySpeedDelivered = 0.0
        self.rotDelivered = 0.0

        self.magLimiter = SlewRateLimiter(DriveConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DriveConstants.kRotationalSlewRate)
        self.prevTime = Timer.getFPGATimestamp()

        # The next attributes are set depending on if vision is upported for tracking the robot pose
        self.odometry = None
        self.pose_estimator = None
        self.inst = None

        if not self.vision_odometry or RobotBase.isSimulation():
            # The robots movements are commanded based on the fixed coordinate system of the competition field
            self.field_relative = True

            # Odometry class for tracking robot pose
            self.odometry = SwerveDrive4Odometry(DriveConstants.kDriveKinematics,
                                                 Rotation2d(),
                                                 (self.frontLeft.getPosition(),
                                                  self.frontRight.getPosition(),
                                                  self.rearLeft.getPosition(),
                                                  self.rearRight.getPosition()))
            self.odometryHeadingOffset = Rotation2d(0)

            # Register for any changes in alliance before the match starts
            container.register_alliance_change_callback(self._alliance_change)
            self._alliance_change(container.is_red_alliance,
                                  container.alliance_location)
        else:
            # The robots movements are commanded based on the robot's own orientation
            self.field_relative = False
            self._init_vision_odometry()

            # self.field = self.quest_field
            self._robot.field = self.quest_field

    @property
    def counter(self) -> int:
        return self._robot.counter

    @property
    def field(self) -> Field2d:
        return self._robot.field

    @property
    def gyro(self) -> Optional[Union[Any, pigeon2.Pigeon2]]:
        return self._gyro

    @property
    def sim_yaw(self) -> degrees:
        # TODO: Copied from physics.py reconcile with any existing functions in this class
        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            return self._sim_gyro.getDouble("Yaw").get()

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            return self._sim_gyro.get_yaw().value

        raise NotImplementedError(f"Unsupported IMU/Gyro type: {DriveConstants.GYRO_TYPE}")

    @sim_yaw.setter
    def sim_yaw(self, value: degrees) -> None:
        # TODO: Copied from physics.py reconcile with any existing functions in this class
        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            self._sim_gyro.getDouble("Yaw").set(value)

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            self._sim_gyro_state.set_raw_yaw(value)

        else:
            raise NotImplementedError(f"Unsupported IMU/Gyro type: {DriveConstants.GYRO_TYPE}")

    def _init_vision_odometry_photoncam(self):
        # TODO: Below is code from team 2429 where they use vision to estimate position.
        # TODO: May be better derive a new class for this
        # 2024 - orphan the old odometry, now use the vision enabled version of odometry instead
        initialPose = Pose2d(constants.k_start_x,
                             constants.k_start_y,
                             Rotation2d.fromDegrees(self.get_gyro_angle()))

        self.pose_estimator = SwerveDrive4PoseEstimator(DriveConstants.kDriveKinematics,
                                                        Rotation2d.fromDegrees(self.get_gyro_angle()),
                                                        self.get_module_positions(),
                                                        initialPose=initialPose)
        # get poses from NT
        self.inst = ntcore.NetworkTableInstance.getDefault()

        self.pi_subscriber_dicts: List[Dict[str, Union[ntcore.DoubleArraySubscriber, ntcore.DoubleSubscriber]]] = []
        for pi_name in constants.VisionConstants.k_pi_names:
            this_pi_subscriber_dict = {}
            this_pi_subscriber_dict.update({"robot_pose_info_subscriber": self.inst.getDoubleArrayTopic(
                f"vision/{pi_name}/robot_pose_info").subscribe([])})
            this_pi_subscriber_dict.update(
                {"wpinow_time_subscriber": self.inst.getDoubleTopic(f"vision/{pi_name}/wpinow_time").subscribe(0)})
            self.pi_subscriber_dicts.append(this_pi_subscriber_dict)

        # photonvision camera setup
        self.use_photoncam = constants.k_use_photontags  # decide down below in periodic
        if self.use_photoncam:
            # TODO: Code still from team 2429 below
            from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy  # 2025 is first time for us

            # self.photon_name = "Arducam_OV9281_USB_Camera"
            # self.photon_name = "HD_Pro_Webcam_C920"
            self.photon_name = "Geniuscam"

            self.photoncam_arducam_a = PhotonCamera(self.photon_name)
            self.photoncam_target_subscriber = self.inst.getBooleanTopic(
                f'/photonvision/{self.photon_name}/hasTarget').subscribe(False)
            self.photoncam_latency_subscriber = self.inst.getDoubleTopic(
                f'/photonvision/{self.photon_name}/LatencyMillis').subscribe(0)

            # example is cam mounted facing forward, half a meter forward of center, half a meter up from center
            # robot_to_cam_example = wpimath.geometry.Transform3d(wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
            #     wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),)
            robot_to_cam_arducam_a = Transform3d(
                Translation3d(inchesToMeters(10), inchesToMeters(7.75), 0.45),
                Rotation3d.fromDegrees(0.0, 0.0, math.radians(270)))

            robot_to_cam_arducam_a = Transform3d(
                Translation3d(inchesToMeters(0), inchesToMeters(0), 0.),
                Rotation3d.fromDegrees(0.0, -20, math.radians(0)))

            # geniuscam with standard convention - 10.5in x, -8in y, -111 degrees yaw
            robot_to_cam_arducam_a = Transform3d(
                Translation3d(inchesToMeters(0), inchesToMeters(0), 0.),
                Rotation3d.fromDegrees(0.0, 0, math.radians(0)))  # -111 from front if ccw +

            # todo - see if we can update the PoseStrategy based on if disabled, and use closest to current odometry when enabled
            # but MULTI_TAG_PNP_ON_COPROCESSOR probably does not help at all since we only see one tag at a time
            # TODO: we can set a fallback for multi_tag_pnp_on_coprocessor, we can make that lowest ambiguity or cloest to current odo
            self.photoncam_pose_est = PhotonPoseEstimator(
                apriltag.AprilTagFieldLayout.loadField(apriltag.AprilTagField.k2025ReefscapeWelded),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                self.photoncam_arducam_a,
                robot_to_cam_arducam_a)
            # default is above PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, but maybe PoseStrategy.LOWEST_AMBIGUITY is better
            self.photoncam_pose_est.primaryStrategy = PoseStrategy.LOWEST_AMBIGUITY

            # -----------   CJH simple apriltags  ------------
            # get poses from NT
            self.use_CJH_apriltags = constants.k_use_CJH_tags  # down below we decide which one to use in the periodic method
            # lhack turned off 15:48 2/28/25 to test pathplanner wo tags first
            self.inst = ntcore.NetworkTableInstance.getDefault()
            # TODO - make this a loop with just the names
            self.arducam_back_pose_subscriber = self.inst.getDoubleArrayTopic(
                "/Cameras/ArducamBack/poses/tag1").subscribe([0] * 8)
            self.arducam_back_count_subscriber = self.inst.getDoubleTopic(
                "/Cameras/ArducamBack/tags/targets").subscribe(0)

            self.arducam_high_pose_subscriber = self.inst.getDoubleArrayTopic(
                "/Cameras/ArducamHigh/poses/tag1").subscribe([0] * 8)
            self.arducam_high_count_subscriber = self.inst.getDoubleTopic(
                "/Cameras/ArducamHigh/tags/targets").subscribe(0)

            self.genius_low_pose_subscriber = self.inst.getDoubleArrayTopic("/Cameras/GeniusLow/poses/tag1").subscribe(
                [0] * 8)
            self.genius_low_count_subscriber = self.inst.getDoubleTopic("/Cameras/GeniusLow/tags/targets").subscribe(0)

            self.logitech_reef_pose_subscriber = self.inst.getDoubleArrayTopic(
                "/Cameras/LogitechReef/poses/tag1").subscribe([0] * 8)
            self.logitech_reef_count_subscriber = self.inst.getDoubleTopic(
                "/Cameras/LogitechReef/tags/targets").subscribe(0)

            # set myself up for a zip later on
            self.pose_subscribers = [self.arducam_back_pose_subscriber, self.arducam_high_pose_subscriber,
                                     self.genius_low_pose_subscriber, self.logitech_reef_pose_subscriber]
            self.count_subscribers = [self.arducam_back_count_subscriber, self.arducam_high_count_subscriber,
                                      self.genius_low_count_subscriber, self.logitech_reef_count_subscriber]

            self.desired_tags = constants.VisionConstants.k_valid_tags

            # TODO - give me a list of six filters for the apriltags - smooth if we are not moving, else use reset each measurement
            # def tag_filter(window):
            #     return [wpimath.filter.LinearFilter.movingAverage(window) for _ in range(6) ]
            # window = 5
            # self.tag_motion_filters = [tag_filter(window) for _ in self.pose_subscribers]

            robot_config = RobotConfig.fromGUISettings()

            AutoBuilder.configure(
                pose_supplier=self.get_pose,
                reset_pose=self.resetOdometry,
                robot_relative_speeds_supplier=self.get_relative_speeds,
                output=self.drive_robot_relative,
                controller=apriltag.k_pathplanner_holonomic_controller,
                robot_config=robot_config,
                should_flip_path=self.flip_path,
                drive_subsystem=self
            )

            self.automated_path = None

            # QuestNav - eventually we will push all of this out to a class
            # TODO: This needs to be supported eventually (See Team 2429 robot for 2025)
            from helpers.questnav.questnav2 import QuestNav
            self.questnav = QuestNav()
            self.quest_to_robot = Transform2d(inchesToMeters(-8.35), inchesToMeters(-10.50),
                                              Rotation2d().fromDegrees(270))  # 10.50 -8.35
            # This is quest-centric coordinate. X is robot center position -8.35 inch as seen rom Quest. y is robot center -10.50 inches as seen from Quest
            # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
            self.quest_field = Field2d()
            self.quest_has_synched = False  # use this to check in disabled whether to update the quest with the robot odometry
            SmartDashboard.putBoolean('questnav_synched', self.quest_has_synched)
            self.use_quest = constants.k_use_quest_odometry
            SmartDashboard.putBoolean('questnav_in_use', self.use_quest)

            # note - have Risaku standardize these with the rest of the putDatas
            SmartDashboard.putData('QuestResetOdometry',
                                          InstantCommand(lambda: self.quest_reset_odometry()).ignoringDisable(True))
            SmartDashboard.putData('QuestSyncOdometry',
                                          InstantCommand(lambda: self.quest_sync_odometry()).ignoringDisable(True))
            # SmartDashboard.putData('QuestUnSync', InstantCommand(lambda: self.quest_unsync_odometry()).ignoringDisable(True))
            SmartDashboard.putData('QuestEnableToggle',
                                          InstantCommand(lambda: self.quest_enabled_toggle()).ignoringDisable(True))
            SmartDashboard.putData('QuestSyncToggle',
                                          InstantCommand(lambda: self.quest_sync_toggle()).ignoringDisable(True))
            # end of init

    def _alliance_change(self, is_red: bool, location: int) -> None:
        """
        Change in alliance occurred before match started. If simulation is
        supported, then 'physics.py' handles this.
        """
        if RobotBase.isSimulation():
            # Use test subsystem settings if simulation
            initial_pose = RED_TEST_POSE[location] if is_red else BLUE_TEST_POSE[location]
            self.resetOdometry(initial_pose)

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        SmartDashboard.putData("Field", self.field)
        SmartDashboard.putString('Gyro/type', DriveConstants.GYRO_TYPE)

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        divisor = 10 if self._robot.isEnabled() else 20
        update_dash = self._robot.counter % divisor == 0

        if update_dash:
            pose = self.get_pose()  # self.odometry.getPose()
            SmartDashboard.putNumber("Drivetrain/x", pose.x)
            SmartDashboard.putNumber("Drivetrain/y", pose.y)
            SmartDashboard.putNumber("Drivetrain/heading", pose.rotation().degrees())

            SmartDashboard.putNumber('keep_angle', self.keep_angle)

            if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
                SmartDashboard.putNumber('Gyro/yaw', self.get_yaw())
                SmartDashboard.putNumber('Gyro/angle1', self.get_angle())
                SmartDashboard.putNumber('Gyro/angle2', self.get_gyro_angle())

            elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
                SmartDashboard.putNumber('Gyro/yaw', self.get_yaw())
                SmartDashboard.putNumber('Gyro/pitch', self.get_pitch())
                SmartDashboard.putNumber('Gyro/roll', self.get_roll())

    def configure_button_bindings(self, driver, shooter) -> None:
        """
        Configure the driver and shooter joystick controls here
        """
        if self.front_camera is not None and self.self.localizer is not None:
            def turn_to_object() -> None:
                """
                This command is used to have the robot camera

                If you want the robot to slowly chase that object... replace the 'self.rotate'
                line below with: self.arcadeDrive(0.1, turn_speed)

                """
                x = self.front_camera.getX()
                turn_speed = -0.01 * x
                self.rotate(turn_speed)

            # TODO: Make the button assignment come from a constants.py file / list somewhere.
            #       so we can keep track what is assigned
            # button = self.driverController.button(XboxController.Button.kB)
            # button.whileTrue(RunCommand(turn_to_object, self))
            # button.onFalse(InstantCommand(lambda: self.drive(0, 0, 0,

        pass  # TODO: Add me

    def periodic(self) -> None:
        enabled = self._robot.isEnabled()
        log_it = self._robot.counter % 20 == 0 and enabled

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            if not self.gyro_calibrated and not self._gyro.isCalibrating():
                # Gyro has finished calibrating, set it to zero
                logger.info(f"DriveSubsystem:Periodic: Gyro was calibrated at count {self.counter}")
                self.zero_yaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
                self.gyro_calibrated = True


        fl_pos = self.frontLeft.getPosition()
        fr_pos = self.frontRight.getPosition()
        rl_pos = self.rearLeft.getPosition()
        rr_pos = self.rearRight.getPosition()
        heading = self.getGyroHeading()

        if log_it:
            logger.debug(
                f"updating odometry: heading: {heading}, fl_pos: {fl_pos}, fr_pos: {fr_pos}, rl_pos: {rl_pos}, rr_pos: {rr_pos}")
            logger.debug(f"pose before update: {self.getPose()}")

        # Update the odometry in the periodic block
        pose = self.odometry.update(heading, (fl_pos, fr_pos, rl_pos, rr_pos,))

        if log_it:
            logger.debug(
                f"Drive periodic: gyro Heading: {self.getGyroHeading()}, x: {pose.x}, y: {pose.y}, rot: {pose.rotation().degrees()}")

        self.field.setRobotPose(pose)

        # Update SmartDashboard for this subsystem
        self.dashboard_periodic()


    def periodic_other(self) -> None:
        # TODO: Has team 2429 vision support. Keep until we can use it
        # send our current time to the dashboard
        ts = Timer.getFPGATimestamp()
        SmartDashboard.putNumber('_timestamp',
                                 ts)  # this one we actually do every time TODO - see if this is done by wpilib and use it instead

        # use this if we have a phononvision camera - which we don't as of 20250316
        if self.vision_odometry and RobotBase.isReal() and self.use_photoncam:  # sim complains if you don't set up a sim photoncam
            has_photontag = self.photoncam_target_subscriber.get()
            # has_photontag = self.photoncam_target_subscriber.get()
            # how do we get the time offset and standard deviation?

            if has_photontag:  # TODO - CHANGE ANGLE OF CAMERA MOUNTS
                result = self.photoncam_arducam_a.getLatestResult()
                cam_est_pose = self.photoncam_pose_est.update(result)
                # can also use result.hasTargets() instead of nt
                target = result.getBestTarget()
                if target is not None:  # not sure why it returns None sometimes when we have tags
                    # get id with target.fiducialId
                    # get % of camera with target.getArea() to get a sense of distance
                    try:
                        ambiguity = target.getPoseAmbiguity()
                    except AttributeError as e:
                        ambiguity = 999

                    latency = self.photoncam_latency_subscriber.get()
                    # if statements to test if we want to update using a tag
                    use_tag = constants.k_use_photontags  # can disable this in constants
                    # do not allow large jumps when enabled
                    delta_pos = Translation2d.distance(self.get_pose().translation(),
                                                       cam_est_pose.estimatedPose.translation().toTranslation2d())
                    use_tag = False if (
                            delta_pos > 1 and DriverStation.isEnabled()) else use_tag  # no big movements in odometry from tags
                    # limit a pose rotation to less than x degrees
                    delta_rot = math.fabs(
                        self.get_pose().rotation().degrees() - cam_est_pose.estimatedPose.rotation().angle_degrees)
                    use_tag = False if delta_rot > 10 and DriverStation.isEnabled() else use_tag
                    # TODO - ignore tags if we are moving too fast
                    use_tag = False if self._gyro.getRate() > 90 else use_tag  # no more than n degrees per second turning if using a tag
                    use_tag = False if latency > 100 else use_tag  # ignore stale tags

                    # TODO - filter out tags that are too far away from camera (different from pose itself too far away from robot)
                    # filter out tags with too much ambiguity - where ratio > 0.2 per docs
                    use_tag = False if ambiguity > 0.2 else use_tag

                    if use_tag:
                        self.pose_estimator.addVisionMeasurement(cam_est_pose.estimatedPose.toPose2d(), ts - latency,
                                                                 constants.DrivetrainConstants.k_pose_stdevs_large)
                # _ = self.photoncam_arducam_a.getAllUnreadResults()
            else:
                pass

            if self.counter % 10 == 0 and self.use_photoncam:  # get diagnostics on photontags
                SmartDashboard.putBoolean('photoncam_targets_exist', has_photontag)
                if has_photontag:
                    try:
                        ambiguity = self.photoncam_arducam_a.getLatestResult().getBestTarget().getPoseAmbiguity()
                    except AttributeError as e:
                        ambiguity = 998
                    SmartDashboard.putNumber('photoncam_ambiguity', ambiguity)
                else:
                    SmartDashboard.putNumber('photoncam_ambiguity', 997)

        if self.vision_odometry and self.use_quest and self.quest_has_synched and self.counter % 5 == 0:
            # print('quest pose synced')
            quest_accepted = SmartDashboard.getBoolean("QUEST_POSE_ACCEPTED", False)
            quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)
            delta_pos = Translation2d.distance(self.get_pose().translation(), quest_pose.translation())
            if delta_pos < 5 and quest_accepted:  # if the quest is way off, we don't want to update from it
                self.pose_estimator.addVisionMeasurement(quest_pose, Timer.getFPGATimestamp(),
                                                         constants.DrivetrainConstants.k_pose_stdevs_disabled)

        if self.vision_odometry and self.use_CJH_apriltags:  # loop through all of our subscribers above
            for count_subscriber, pose_subscriber in zip(self.count_subscribers, self.pose_subscribers):
                # print(f"count subscriber says it has {count_subscriber.get()} tags")
                if count_subscriber.get() > 0:  # use this camera's tag
                    # update pose from apriltags
                    tag_data = pose_subscriber.get()  # 8 items - timestamp, id, tx ty tx rx ry rz
                    id = tag_data[0]
                    tx, ty, tz = tag_data[2], tag_data[3], tag_data[4]
                    rx, ry, rz = tag_data[5], tag_data[6], tag_data[7]
                    tag_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()

                    use_tag = constants.k_use_CJH_tags  # can disable this in constants
                    # do not allow large jumps when enabled
                    delta_pos = Translation2d.distance(self.get_pose().translation(), tag_pose.translation())
                    # 20251018 commented out the 1m sanity check in case the questnav dies - this way we can get back
                    # use_tag = False if (delta_pos > 1 and DriverStation.isEnabled()) else use_tag  # no big movements in odometry from tags
                    use_tag = False if self._gyro.getRate() > 90 else use_tag  # no more than n degrees per second turning if using a tag
                    # use_tag = False if id not in self.desired_tags else use_tag

                    # TODO - figure out ambiguity (maybe pass to NT from the pi)
                    # do i have a fatal lag issue?  am i better without the time estimate?
                    # based on https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/13
                    # I gave a fairly high x and y, and a very high theta
                    if use_tag:
                        # print(f'adding vision measurement at {wpilib.getTime()}')
                        sdevs = constants.DrivetrainConstants.k_pose_stdevs_large if DriverStation.isEnabled() else constants.DrivetrainConstants.k_pose_stdevs_disabled
                        self.pose_estimator.addVisionMeasurement(tag_pose, tag_data[0], sdevs)

        # Update the odometry in the periodic block -
        if self.vision_odometry and RobotBase.isReal():
            # self.odometry.update(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),)
            self.pose_estimator.updateWithTime(Timer.getFPGATimestamp(),
                                               Rotation2d.fromDegrees(self.get_gyro_angle()),
                                               self.get_module_positions())

        # in sim, we update from physics.py
        # TODO: if we want to be cool and have spare time, we could use SparkBaseSim with FlywheelSim to do
        # actual physics simulation on the swerve modules instead of assuming perfect behavior

        if self.vision_odometry and self.counter % 10 == 0:
            pose = self.get_pose()  # self.odometry.getPose()
            if True:  # RobotBase.isReal():  # update the NT with odometry for the dashboard - sim will do its own
                SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
                SmartDashboard.putNumber('drive_x', pose.X())
                SmartDashboard.putNumber('drive_y', pose.Y())
                SmartDashboard.putNumber('drive_theta', pose.rotation().degrees())

            # SmartDashboard.putNumber('_gyro', self.get_angle())   # TODO: Support
            # SmartDashboard.putNumber('_gyro_yaw', self.get_yaw())
            # SmartDashboard.putNumber('_gyro_angle', self.get_gyro_angle())

            SmartDashboard.putNumber('keep_angle', self.keep_angle)
            # SmartDashboard.putNumber('keep_angle_output', output)

            # post yaw, pitch, roll so we can see what is going on with the climb
            # ypr = [self._gyro.getYaw(), self.get_pitch(), self._gyro.getRoll(), self._gyro.getRotation2d().degrees()]
            # SmartDashboard.putNumberArray('_gyro_YPR', ypr)

            # monitor power as well
            if True:  # RobotBase.isReal():
                # there's some kind of voltage simulation but idk if this covers it
                voltage = self.pdh.getVoltage()
                total_current = self.pdh.getTotalCurrent()
            else:
                # make up a current based on how fast we're going
                total_current = 2 + 10 * sum(
                    [math.fabs(module.drivingEncoder.getVelocity()) for module in self.swerve_modules])
                voltage = 12.5 - 0.02 * total_current

            SmartDashboard.putNumber('_pdh_voltage', voltage)
            SmartDashboard.putNumber('_pdh_current', total_current)

            if constants.k_swerve_debugging_messages:  # this is just a bit much unless debugging the swerve
                angles = [m.turningEncoder.getPosition() for m in self.swerve_modules]
                absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
                for idx, absolute in enumerate(absolutes):
                    SmartDashboard.putNumber(f"absolute {idx}", absolute)

                SmartDashboard.putNumberArray(f'_angles', angles)
                # SmartDashboard.putNumberArray(f'_analog_radians', absolutes)

        # Import pose from QuestNav.
        if self.vision_odometry:
            self.quest_periodic()

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller
        self.kinematics: SwerveDrive4Kinematics = DriveConstants.kDriveKinematics  # our swerve drive kinematics

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            # NavX (SPI interface)
            self._sim_gyro = simulation.SimDeviceSim("navX-Sensor[4]")

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            self._sim_gyro = self.gyro
            self._sim_gyro_state = self.gyro.sim_state

        else:
            raise NotImplementedError(f"Unsupported IMU/Gyro type: {DriveConstants.GYRO_TYPE}")

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
            spark = simulation.SimDeviceSim(f'SPARK MAX [{can_id}]')

            self.spark_dict[spark_name] = {
                'controller': spark,
                'position': spark.getDouble('Position'),
                'velocity': spark.getDouble('Velocity'),
                'output': spark.getDouble('Applied Output')
            }

        # set up the initial location of the robot on the field
        self._alliance_change(self._container.is_red_alliance,
                              self._container.alliance_location)

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

        now, tm_diff = kwargs["now"], kwargs["tm_diff"]
        amperes_used = 0.0  # TODO: Support in future

        log_it = self._robot.counter % 20 == 0

        if log_it:
            logger.debug("Update swerve:===========================================")
            logger.debug(f"Update swerve: Entry. tm_diff: {tm_diff:.4f}")

        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        target_angles = [SmartDashboard.getNumberArray(dash_value, [0, 0])[1] for dash_value in dash_values]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)  # this works to update the simulated spark

        if self._robot.counter % 10 == 0 and self._robot.isEnabled():
            SmartDashboard.putNumberArray('target_angles', target_angles)

        # using our own kinematics to update the chassis speeds
        module_states = self.get_desired_swerve_module_states()
        speeds = self.kinematics.toChassisSpeeds(module_states)

        if log_it:
            logger.debug(f"Update swerve before drive command: module states: {module_states}, speeds: {speeds}")

        # update the sim's robot. Returned value is same as what is returned from self._physics_controller.get_pose()
        pose = self._physics_controller.drive(speeds, tm_diff)

        # Limit it to the field size (manually)  TODO: Is there a programmatic way to do this?
        x = min(17.5, max(0.0, pose.x))
        y = min(8.0, max(0.0, pose.y))
        new_pose = Pose2d(x, y, pose.rotation())

        self.resetSimPose(new_pose, [SwerveModulePosition()] * 4,
                          self._physics_controller.get_pose().rotation())
        previous = self.sim_yaw
        omega = speeds.omega
        gyro_degrees = math.degrees(speeds.omega * tm_diff)
        new = previous - math.degrees(speeds.omega * tm_diff)

        if log_it:
            logger.debug(f"Update swerve: previous: {previous}, new: {new}, omega: {omega}, degrees: {gyro_degrees}")

        gyro_degrees = pose.rotation().degrees()
        self.sim_yaw = -gyro_degrees if DriveConstants.kGyroReversed else gyro_degrees

        return amperes_used

    def getHeading(self) -> Rotation2d:
        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    get_pose = getPose  # Alias

    def zero_yaw(self):

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            self._gyro.zeroYaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating

            if RobotBase.isSimulation():
                gyro = simulation.SimDeviceSim("navX-Sensor[4]")
                gyro_yaw = gyro.getDouble("Yaw")  # for some reason it seems we have to set Yaw and not Angle
                # gyro_angle = gyro.getDouble("Angle")
                # gyro_angle.set(0.0)
                gyro_yaw.set(0.0)

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            self._gyro.set_yaw(0.0)  # we boot up at zero degrees  - note - you can't reset this while calibrating

            if RobotBase.isSimulation():
                print("TODO: Need to zero simulation gyro")  # TODO

    def resetSimPose(self, pose, wheel_positions, rotation):
        """
        Currently just used in simulation
        """
        if RobotBase.isSimulation():
            if self.vision_odometry:
                self.pose_estimator.resetPosition(gyroAngle=rotation, wheelPositions=wheel_positions, pose=pose)
            else:
                self.odometry.resetPosition(gyroAngle=rotation, wheelPositions=wheel_positions, pose=pose)

    def resetOdometry(self, pose: Pose2d, resetGyro=True) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        if self.vision_odometry:
            self.pose_estimator.resetPosition(Rotation2d.fromDegrees(self.get_gyro_angle()),
                                              self.get_module_positions(),
                                              pose)
            return

        if resetGyro:
            self._gyro.reset()
            # self._gyro.setAngleAdjustment(0)   # TODO: Look into this?
            self._lastGyroAngleAdjustment = 0
            self._lastGyroAngleTime = 0
            self._lastGyroAngle = 0

        positions = (self.frontLeft.getPosition(), self.frontRight.getPosition(),
                     self.rearLeft.getPosition(), self.rearRight.getPosition())

        self.odometry.resetPosition(self.getGyroHeading(), positions, pose)

        self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()

    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)

        positions = (self.frontLeft.getPosition(), self.frontRight.getPosition(),
                     self.rearLeft.getPosition(), self.rearRight.getPosition())

        self.odometry.resetPosition(pose.rotation() - self.odometryHeadingOffset, positions, newPose)
        self.odometryHeadingOffset += dRot

    def stop(self):
        self.arcadeDrive(0, 0)

    def arcadeDrive(self, xSpeed: float, rot: float, assumeManualInput: bool = False) -> None:
        self.drive(xSpeed, 0, rot, False, False, square=assumeManualInput)

    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param rotSpeed: rotation speed
        """
        self.arcadeDrive(0, rotSpeed)

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool,
              rateLimit: bool, square: bool = False) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        :param square:        Whether to square the inputs (useful for manual control)
        """
        if square:
            rot = rot * abs(rot)
            norm = math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed)
            xSpeed = xSpeed * norm
            ySpeed = ySpeed * norm

        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if rateLimit:
            # Convert XY to polar for rate limiting
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.hypot(xSpeed, ySpeed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(DriveConstants.kDirectionSlewRate / self.currentTranslationMag)
            else:
                directionSlewRate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            currentTime = Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.prevTime
            angleDif = swerveutils.angleDifference(inputTranslationDir, self.currentTranslationDir)

            if angleDif < 0.45 * math.pi:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(inputTranslationMag)

            elif angleDif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.currentTranslationMag > 1e-4:
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)
                else:
                    self.currentTranslationDir = swerveutils.wrapAngle(self.currentTranslationDir + math.pi)
                    self.currentTranslationMag = self.magLimiter.calculate(inputTranslationMag)

            else:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(0.0)

            self.prevTime = currentTime

            xSpeedCommanded = self.currentTranslationMag * math.cos(self.currentTranslationDir)
            ySpeedCommanded = self.currentTranslationMag * math.sin(self.currentTranslationDir)
            self.currentRotation = self.rotLimiter.calculate(rot)

        else:
            self.currentRotation = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        self.xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        self.ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond
        self.rotDelivered = self.currentRotation * DriveConstants.kMaxAngularSpeed

        if fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeedDelivered, self.ySpeedDelivered,
                                                           self.rotDelivered, self.getGyroHeading())
        else:
            speeds = ChassisSpeeds(self.xSpeedDelivered, self.ySpeedDelivered, self.rotDelivered)

        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds)

        maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond
        if self.maxSpeedScaleFactor is not None:
            maxSpeed = maxSpeed * self.maxSpeedScaleFactor()

        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed)
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(self, desiredStates: SwerveModuleStats) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond
        if self.maxSpeedScaleFactor is not None:
            maxSpeed = maxSpeed * self.maxSpeedScaleFactor()
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, maxSpeed)
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.rearLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.rearRight.resetEncoders()

    def getGyroHeading(self) -> Rotation2d:
        """
        Returns the heading of the robot, tries to be smart when gyro is disconnected
        """
        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
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

                raw_angle = self.get_raw_angle()
                gyroAngle = -raw_angle if DriveConstants.kGyroReversed else raw_angle

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

                if self.counter % 10 == 0 and self._robot.isEnabled():
                    logger.debug(f"Gyro: angle: {gyroAngle}")

            if state != self._lastGyroState:
                SmartDashboard.putString("gyro", f"{state} after {int(now - past)}s")
                self._lastGyroState = state

            last_angle = self._lastGyroAngle

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            last_angle = self.get_yaw()

        else:
            raise NotImplementedError(f"IMU/Gyro type of {DriveConstants.GYRO_TYPE} not implemented")

        return Rotation2d.fromDegrees(last_angle)

    def getTurnRateDegreesPerSec(self) -> degrees_per_second:
        """
        Returns the turn rate of the robot (in degrees per second)
        """
        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            rate = self._gyro.getRate()

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            rate = self._gyro.get_angular_velocity_z_world().value

        else:
            raise NotImplementedError(f"IMU/Gyro type of {DriveConstants.GYRO_TYPE} not implemented")

        return -rate if DriveConstants.kGyroReversed else rate

    ##########################################################
    # TODO: All the following are related to team 2429 and pathplanner. These have not been tested and
    #       we may need to refactor the 'drive()' method above

    #  -------------  THINGS PATHPLANNER NEEDS  - added for pathplanner 20230218 CJH
    def get_relative_speeds(self) -> ChassisSpeeds:
        # added for pathplanner 20230218 CJH
        return DriveConstants.kDriveKinematics.toChassisSpeeds(self.get_module_states())

    def drive_robot_relative(self, chassis_speeds: ChassisSpeeds, feedforwards):
        """c
        feedforwards isn't used at all so pass it whatever
        """
        # required for the pathplanner lib's path following based on chassis speeds
        # idk if we need the feedforwards
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates,
                                                                          DriveConstants.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)

    def flip_path(self):  # pathplanner needs a function to see if it should mirror a path
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return False
        else:
            return True

    # def follow_pathplanner_trajectory_command(self, trajectory:PathPlannerTrajectory, is_first_path:bool):
    #     #from pathplannerlib.path import PathPlannerPath
    #     #from pathplannerlib.commands import FollowPathWithEvents, FollowPathHolonomic
    #     #from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants

    #     # copy of pathplannerlib's method for returning a swervecommand, with an optional odometry reset
    #     # using the first pose of the trajectory
    #     if is_first_path:
    #         reset_cmd = commands2.InstantCommand(self.resetOdometry(trajectory.getInitialTargetHolonomicPose()))
    #     else:
    #         reset_cmd = commands2.InstantCommand()

    #     # useful stuff controller.PPHolonomicDriveController, controller.PIDController, auto.FollowPathHolonomic
    #     swerve_controller_cmd = None

    #     cmd = commands2.SequentialCommandGroup(reset_cmd, swerve_controller_cmd)

    #     return cmd

    # -------------- END PATHPLANNER STUFF

    def reset_keep_angle(self) -> None:
        """
        perhaps deprecated because we want to use resetOdometry to reset the gyro
        """
        self.last_rotation_time = self.keep_angle_timer.get()  # reset the rotation time
        self.last_drive_time = self.keep_angle_timer.get()  # reset the drive time

        new_angle = self.get_angle()
        print(f'  resetting keep angle from {self.keep_angle:.1f} to {new_angle:.1f}', flush=True)
        self.keep_angle = new_angle

    def perform_keep_angle(self, xSpeed, ySpeed,
                           rot):  # update rotation if we are drifting when trying to drive straight
        output = rot  # by default we will return rot unless it needs to be changed
        if math.fabs(rot) > DriveConstants.k_inner_deadband:  # we are actually intending to rotate
            self.last_rotation_time = self.keep_angle_timer.get()
        if math.fabs(xSpeed) > DriveConstants.k_inner_deadband or math.fabs(ySpeed) > DriveConstants.k_inner_deadband:
            self.last_drive_time = self.keep_angle_timer.get()

        self.time_since_rotation = self.keep_angle_timer.get() - self.last_rotation_time
        self.time_since_drive = self.keep_angle_timer.get() - self.last_drive_time

        if self.time_since_rotation < 0.5:  # (update keep_angle until 0.5s after rotate command stops to allow rotate to finish)
            self.keep_angle = self.get_angle()  # todo: double check SIGN (and units are in degrees)
        elif math.fabs(
                rot) < DriveConstants.k_inner_deadband and self.time_since_drive < 0.25:  # stop keep_angle .25s after you stop driving
            # output = self.keep_angle_pid.calculate(-self.get_angle(), self.keep_angle)  # 2023
            # TODO: figure out if we want YAW or ANGLE, and WHY NOT BE CONSISTENT WITH YAW AND ANGLE?
            output = self.keep_angle_pid.calculate(self.get_angle(),
                                                   self.keep_angle)  # 2024 real, can we just use YAW always?
            output = output if math.fabs(output) < 0.2 else 0.2 * math.copysign(1, output)  # clamp at 0.2

        if self.counter % 20 == 0:
            SmartDashboard.putNumber('keep_angle_output', output)

        return output

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        angles = [45, -45, -45, 45]

        for angle, swerve_module in zip(angles, self.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(angle)))

    def set_straight(self) -> None:
        """Sets the wheels straight so we can push the robot."""
        angles = [0, 0, 0, 0]
        for angle, swerve_module in zip(angles, self.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(angle)))

    def setModuleStates(self, desiredStates: Tuple[SwerveModuleState]) -> None:
        """Sets the swerve ModuleStates.
        :param desiredStates: The desired SwerveModule states.
        """
        desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxTotalSpeed)
        for idx, m in enumerate(self.swerve_modules):
            m.setDesiredState(desiredStates[idx])

    def setDesiredTags(self, desired_tags: List[int]) -> None:
        self.desired_tags = desired_tags

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        [m.resetEncoders() for m in self.swerve_modules]

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self._gyro.reset()

    zeroGyro = zeroHeading  # Alias

    def lock(self) -> None:
        print("What does: swerveDrive.lockPose() do")

    def resetGyroToInitial(self) -> None:
        print(""" What does this do
        {
            zeroGyro();
        swerveDrive.setGyroOffset(new
        Rotation3d());
        }
        """)

    def setMotorBrake(self, brake: bool) -> None:
        # TODO: Need to actually set the IdleMode to 'brake' since this
        #       would be useful on an incline as well
        if brake:
            for motor in (self.rearLeft, self.rearRight, self.frontLeft, self.frontRight):
                motor.stop()

    def get_module_positions(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getPosition() for m in self.swerve_modules]

    def get_module_states(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getState() for m in self.swerve_modules]

    def get_raw_angle(self) -> degrees:  # never reversed value for using PIDs on the heading
        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            return self._gyro.getYaw()

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            return self._gyro.getAngle()

        raise NotImplementedError(f"IMU/Gyro type of {DriveConstants.GYRO_TYPE} not implemented")

    def get_gyro_angle(self) -> degrees:
        # note this does add in the current offset
        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            angle = self._gyro.getAngle()
            return -angle if DriveConstants.kGyroReversed else angle

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            return self.get_yaw()

        raise NotImplementedError(f"IMU/Gyro type of {DriveConstants.GYRO_TYPE} not implemented")

    def get_angle(self) -> degrees:
        return self.get_pose().rotation().degrees()

    def get_yaw(self) -> degrees:  # helpful for determining nearest heading parallel to the wall
        # but you should probably never use this - just use get_angle to be consistent
        # because yaw does NOT return the offset that get_Angle does

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            yaw = self._gyro.getYaw()

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            yaw = self._gyro.get_yaw().value

        else:
            raise NotImplementedError(f"IMU/Gyro type of {DriveConstants.GYRO_TYPE} not implemented")

        return -yaw if DriveConstants.kGyroReversed else yaw

    def get_pitch(self) -> degrees:  # need to calibrate the gyro?
        pitch_offset = 0  # TODO: Always zero?

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            return self._gyro.getPitch() - pitch_offset

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            return self._gyro.get_pitch().value - pitch_offset

        else:
            raise NotImplementedError(f"IMU/Gyro type of {DriveConstants.GYRO_TYPE} not implemented")

    def get_roll(self) -> degrees:  # need to calibrate the gyro?
        roll_offset = 0  # TODO: Always zero?

        if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX:
            return self._gyro.getRoll() - roll_offset

        elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
            return self._gyro.get_roll().value - roll_offset

        else:
            raise NotImplementedError(f"IMU/Gyro type of {DriveConstants.GYRO_TYPE} not implemented")

    def reset_gyro(self, adjustment=None) -> None:  # use this from now on whenever we reset the gyro
        """
        perhaps deprecated because we want to use resetOdometry to reset the gyro 1/12/25 LHACK
        """
        self._gyro.reset()
        if adjustment is not None:
            # ADD adjustment - e.g trying to update the gyro from a pose
            self._gyro.setAngleAdjustment(adjustment)
        else:
            # make sure there is no adjustment
            self._gyro.setAngleAdjustment(0)

        self.reset_keep_angle()

    # figure out the nearest stage - or any tag, I suppose if we pass in a list
    def get_nearest_tag(self, destination='stage'):
        # get a field so we can query the tags
        field = apriltag.AprilTagField.k2025ReefscapeWelded
        layout = apriltag.AprilTagFieldLayout.loadField(field)
        current_pose = self.get_pose()

        if destination == 'reef':
            # get all distances to the stage tags
            tags = [6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21,
                    22]  # the ones we can see from driver's station - does not matter if red or blue
            x_offset, y_offset = -0.10, 0.10  # subtracting translations below makes +x INTO the tage, +y LEFT of tag
            robot_offset = Pose2d(Translation2d(x_offset, y_offset), Rotation2d(0))
            face_tag = True  # do we want to face the tag?
        else:
            raise ValueError('  location for get_nearest tag must be in ["stage", "amp"] etc')

        poses = [layout.getTagPose(tag).toPose2d() for tag in tags]
        distances = [current_pose.translation().distance(pose.translation()) for pose in poses]

        # sort the distances
        combined = list(zip(tags, distances))
        combined.sort(key=lambda x: x[1])  # sort on the distances
        sorted_tags, sorted_distances = zip(*combined)
        nearest_pose = layout.getTagPose(sorted_tags[0])  # get the pose of the nearest stage tag

        # transform the tag pose to our specific needs
        tag_pose = nearest_pose.toPose2d()  # work with a 2D pose
        tag_rotation = tag_pose.rotation()  # we are either going to match this or face opposite
        robot_offset_corrected = robot_offset.rotateBy(tag_rotation)  # rotate our offset so we align with the tag
        updated_translation = tag_pose.translation() - robot_offset_corrected.translation()  # careful with these signs
        updated_rotation = tag_rotation + Rotation2d(math.pi) if face_tag else tag_rotation  # choose if we flip
        updated_pose = Pose2d(translation=updated_translation, rotation=updated_rotation)  # drive to here

        print(f'  nearest {destination} is tag {sorted_tags[0]} at {nearest_pose.translation()}')
        return sorted_tags[0]  # changed this in 2025 instead of updated_pose

    def get_desired_swerve_module_states(self) -> List[SwerveModuleState]:
        """
        what it says on the wrapper; it's for physics.py because I don't like relying on an NT entry
        to communicate between them (it's less clear what the NT entry is there for, I think) LHACK 1/12/25
        """
        return [module.getDesiredState() for module in self.swerve_modules]

    def quest_periodic(self) -> None:
        if not self.vision_odometry:
            return

        self.questnav.command_periodic()
        quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)
        self.quest_field.setRobotPose(quest_pose)

        if self.counter % 10 == 0:
            SmartDashboard.putData("QUEST_FIELD", self.quest_field)
            if 0 < quest_pose.x < 17.658 and 0 < quest_pose.y < 8.131 and self.questnav.is_connected():
                SmartDashboard.putBoolean("QUEST_POSE_ACCEPTED", True)
                # print("Quest Timestamp: " + str(self.questnav.get_app_timestamp()))
                # print("System Timestamp: " + str(utils.get_system_time_seconds()))
                # if abs(self.questnav.get_data_timestamp() - utils.get_current_time_seconds()) < 5:
                #     print("Timestamp in correct epoch.")
                # self.add_vision_measurement(quest_pose,
                #                            utils.fpga_to_current_time(self.questnav.get_data_timestamp()),
                #                            (0.02, 0.02, 0.035))
            else:
                SmartDashboard.putBoolean("QUEST_POSE_ACCEPTED", False)

            SmartDashboard.putString("QUEST_POSE", str(quest_pose))
            SmartDashboard.putBoolean("QUEST_CONNECTED", self.questnav.is_connected())
            SmartDashboard.putBoolean("QUEST_TRACKING", self.questnav.is_tracking())
            SmartDashboard.putNumber("Quest_Battery_%", self.questnav.get_battery_percent())
            SmartDashboard.putNumber("Quest_Latency", self.questnav.get_latency())
            SmartDashboard.putNumber("Quest_Tracking_lost_count", self.questnav.get_tracking_lost_counter())
            SmartDashboard.putNumber("Quest_Latency", self.questnav.get_latency())
            SmartDashboard.putNumber("Quest_frame_count", self.questnav.get_frame_count())

    def reset_pose_with_quest(self, pose: Pose2d) -> None:
        # self.reset_pose(pose)
        self.questnav.set_pose(pose.transformBy(self.quest_to_robot.inverse()))

    def quest_reset_odometry(self) -> None:
        """Reset robot odometry at the Subwoofer."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            # self.reset_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)))
            # self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))
            self.questnav.set_pose(
                Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)).transformBy(self.quest_to_robot.inverse()))
            # print("reset to red")
        else:
            # self.reset_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)))
            # self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            self.questnav.set_pose(
                Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)).transformBy(self.quest_to_robot.inverse()))
            # print(self.questnav.get_pose())
        print(f"Reset questnav at {Timer.getFPGATimestamp():.1f}s")
        self.quest_unsync_odometry()

    def quest_sync_odometry(self) -> None:
        # self.questnav.set_pose(self.get_pose())
        self.quest_has_synched = True  # let the robot know we have been synched so we don't automatically do it again
        self.questnav.set_pose(self.get_pose().transformBy(self.quest_to_robot.inverse()))
        print(f'Synched quest at {Timer.getFPGATimestamp():.1f}s')
        SmartDashboard.putBoolean('questnav_synched', self.quest_has_synched)

    def quest_unsync_odometry(self) -> None:
        # self.questnav.set_pose(self.get_pose())
        self.quest_has_synched = False  # let the robot know we have been synched so we don't automatically do it again
        print(f'Unsynched quest at {Timer.getFPGATimestamp():.1f}s')
        SmartDashboard.putBoolean('questnav_synched', self.quest_has_synched)

    def quest_enabled_toggle(self, force=None):  # allow us to stop using quest if it is a problem - 20251014 CJH
        if force is None:
            self.use_quest = not self.use_quest  # toggle the boolean
        elif force == 'on':
            self.use_quest = True
        elif force == 'off':
            self.use_quest = False
        else:
            self.use_quest = False

        print(f'swerve use_quest updated to {self.use_quest} at {Timer.getFPGATimestamp():.1f}s')
        SmartDashboard.putBoolean('questnav_in_use', self.use_quest)

    def quest_sync_toggle(self, force=None):  # toggle sync state for dashboard - 20251014 CJH
        current_state = self.quest_has_synched
        if force is None:
            current_state = not current_state  # toggle the boolean
        elif force == 'on':
            current_state = True
        elif force == 'off':
            current_state = False
        else:
            current_state = False

        if current_state:
            self.quest_sync_odometry()
        else:
            self.quest_unsync_odometry()
        # reporting done by the sync/unsync functions

    def is_quest_enabled(self):
        return self.use_quest
