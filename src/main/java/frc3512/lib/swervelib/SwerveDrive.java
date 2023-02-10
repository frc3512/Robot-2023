package frc3512.lib.swervelib;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc3512.lib.logging.SpartanDoubleArrayEntry;
import frc3512.lib.logging.SpartanStringEntry;
import frc3512.lib.swervelib.imu.SwerveIMU;
import frc3512.lib.swervelib.math.SwerveKinematics2;
import frc3512.lib.swervelib.math.SwerveModuleState2;
import frc3512.lib.swervelib.parser.SwerveControllerConfiguration;
import frc3512.lib.swervelib.parser.SwerveDriveConfiguration;
import frc3512.robot.Robot;
import java.util.ArrayList;
import java.util.List;

public class SwerveDrive {

  //
  // Swerve base kinematics object
  public final SwerveKinematics2 kinematics;
  public final SwerveDriveConfiguration swerveDriveConfiguration;
  private final SwerveModule[] swerveModules;
  private final SwerveDrivePoseEstimator odometry;
  public Field2d field = new Field2d();
  public SwerveController swerveController;
  private SwerveIMU imu;
  private double angle, lastTime;
  private Timer timer;
  private SpartanDoubleArrayEntry moduleSpeedSetpointEntry,
      moduleAngleSetpointEntry,
      moduleStateEntry;
  private SpartanStringEntry robotVelocityEntry;

  /**
   * Creates a new swerve drivebase subsystem. Robot is controlled via the drive() method, or via
   * the setModuleStates() method. The drive() method incorporates kinematics— it takes a
   * translation and rotation, as well as parameters for field-centric and closed-loop velocity
   * control. setModuleStates() takes a list of SwerveModuleStates and directly passes them to the
   * modules. This subsystem also handles odometry.
   */
  public SwerveDrive(
      SwerveDriveConfiguration config, SwerveControllerConfiguration controllerConfig) {
    swerveDriveConfiguration = config;
    swerveController = new SwerveController(controllerConfig);
    // Create Kinematics from swerve module locations.
    kinematics = new SwerveKinematics2(config.moduleLocationsMeters);

    // Create an integrator for angle if the robot is being simulated to emulate an IMU
    // If the robot is real, instantiate the IMU instead.
    if (!Robot.isReal()) {
      timer = new Timer();
      timer.start();
      lastTime = 0;
    } else {
      imu = config.imu;
      imu.factoryDefault();
    }

    this.swerveModules = config.modules;

    odometry =
        new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), new Pose2d());
    zeroGyro();

    robotVelocityEntry = new SpartanStringEntry("/Diagnostics/Swerve/Robot Velocity");
    moduleSpeedSetpointEntry =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Module Speed Setpoints");
    moduleAngleSetpointEntry =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Module Angle Setpoints");
    moduleStateEntry = new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Module States");
  }

  /**
   * The primary method for controlling the drivebase. Takes a Translation2d and a rotation rate,
   * and calculates and commands module states accordingly. Can use either open-loop or closed-loop
   * velocity control for the wheel velocities. Also has field- and robot-relative modes, which
   * affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   * @param isOpenLoop Whether to use closed-loop velocity control. Set to true to disable
   *     closed-loop.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    // Creates a robot-relative ChassisSpeeds object, converting from field-relative speeds if
    // necessary.
    ChassisSpeeds velocity =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    // Display commanded speed for testing
    robotVelocityEntry.set(velocity.toString());

    // Calculate required module states via kinematics
    SwerveModuleState2[] swerveModuleStates = kinematics.toSwerveModuleStates(velocity);

    setModuleStates(swerveModuleStates, isOpenLoop);
  }

  /**
   * Set the module states (azimuth and velocity) directly. Used primarily for auto pathing.
   *
   * @param desiredStates A list of SwerveModuleStates to send to the modules.
   * @param isOpenLoop Whether to use closed-loop velocity control. Set to true to disable
   *     closed-loop.
   */
  public void setModuleStates(SwerveModuleState2[] desiredStates, boolean isOpenLoop) {
    double[] speedEntryValues = new double[4];
    double[] angleEntryValues = new double[4];

    // Desaturates wheel speeds
    SwerveKinematics2.desaturateWheelSpeeds(desiredStates, swerveDriveConfiguration.maxSpeed);

    // Sets states
    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false); // Todo: Send isOpenLoop
      speedEntryValues[module.moduleNumber] =
          desiredStates[module.moduleNumber].speedMetersPerSecond;
      angleEntryValues[module.moduleNumber] = desiredStates[module.moduleNumber].angle.getDegrees();
    }

    moduleSpeedSetpointEntry.set(speedEntryValues);
    moduleAngleSetpointEntry.set(angleEntryValues);
  }

  /**
   * Set field-relative chassis speeds with closed-loop velocity control.
   *
   * @param chassisSpeeds Field-relative.
   */
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    setModuleStates(
        kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getYaw())),
        false);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
    // but not the reverse.  However, because this transform is a simple rotation, negating the
    // angle
    // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        kinematics.toChassisSpeeds(getStates()), getYaw().unaryMinus());
  }

  /**
   * Gets the current robot-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current robot-relative velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return kinematics.toChassisSpeeds(getStates());
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param pose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Adds vision measurements into the pose estimator for better accuracy.
   *
   * @param pose The pose from computer vision to set the odometry to.
   * @param timestamp Current timestamp from computer vision
   */
  public void addVisionMeasurement(Pose2d pose, double timestamp) {
    odometry.addVisionMeasurement(pose, timestamp);
  }

  /**
   * Gets the current module states (azimuth and velocity)
   *
   * @return A list of SwerveModuleStates containing the current module states
   */
  public SwerveModuleState2[] getStates() {
    SwerveModuleState2[] states = new SwerveModuleState2[swerveDriveConfiguration.moduleCount];
    for (SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }
    return states;
  }

  /**
   * Gets the current module positions (azimuth and wheel position (meters))
   *
   * @return A list of SwerveModulePositions containg the current module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions =
        new SwerveModulePosition[swerveDriveConfiguration.moduleCount];
    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }
    return positions;
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    // Resets the real gyro or the angle accumulator, depending on whether the robot is being
    // simulated
    if (Robot.isReal()) {
      imu.setYaw(0);
    } else {
      angle = 0;
    }
    swerveController.lastAngle = 0;
    resetOdometry(new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the imu. CCW positive, not wrapped.
   *
   * @return The yaw angle
   */
  public Rotation2d getYaw() {
    // Read the imu if the robot is real or the accumulator if the robot is simulated.
    if (Robot.isReal()) {
      double[] ypr = new double[3];
      imu.getYawPitchRoll(ypr);
      return Rotation2d.fromDegrees(swerveDriveConfiguration.invertedIMU ? 360 - ypr[0] : ypr[0]);
    } else {
      return new Rotation2d(angle);
    }
  }

  /**
   * Sets the drive motors to brake/coast mode.
   *
   * @param brake True to set motors to brake mode, false for coast.
   */
  public void setMotorBrake(boolean brake) {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setMotorBrake(brake);
    }
  }

  /** Point all modules toward the robot center, thus making the robot very difficult to move. */
  public void setDriveBrake() {
    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.setDesiredState(
          new SwerveModuleState2(0, swerveModule.configuration.moduleLocation.getAngle(), 0), true);
    }
  }

  /**
   * Get the swerve module poses and on the field relative to the robot.
   *
   * @param robotPose Robot pose.
   * @return Swerve module poses.
   */
  public Pose2d[] getSwerveModulePoses(Pose2d robotPose) {
    Pose2d[] poseArr = new Pose2d[swerveDriveConfiguration.moduleCount];
    List<Pose2d> poses = new ArrayList<>();
    for (SwerveModule module : swerveModules) {
      poses.add(
          robotPose.plus(
              new Transform2d(module.configuration.moduleLocation, module.getState().angle)));
    }
    return poses.toArray(poseArr);
  }

  /** Update odometry should be run every loop. */
  public void updateOdometry() {
    // Update odometry
    odometry.update(getYaw(), getModulePositions());

    // Update angle accumulator if the robot is simulated
    if (!Robot.isReal()) {
      angle +=
          kinematics.toChassisSpeeds(getStates()).omegaRadiansPerSecond * (timer.get() - lastTime);
      lastTime = timer.get();
      field.getObject("XModules").setPoses(getSwerveModulePoses(odometry.getEstimatedPosition()));
    }

    field.setRobotPose(odometry.getEstimatedPosition());
    SmartDashboard.putData("Field", field);

    double[] moduleStates = new double[8];
    for (SwerveModule module : swerveModules) {
      moduleStates[module.moduleNumber] = module.getState().angle.getDegrees();
      moduleStates[module.moduleNumber + 1] = module.getState().speedMetersPerSecond;
    }
    moduleStateEntry.set(moduleStates);
  }
}
