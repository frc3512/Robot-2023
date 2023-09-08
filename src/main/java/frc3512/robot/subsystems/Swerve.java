package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanEntryManager;
import frc3512.robot.Constants;
import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

  private final SwerveDrive swerve;
  private final Vision vision;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(Units.feetToMeters(14.5));

  /** Subsystem class for the swerve drive. */
  public Swerve(Vision vision) {
    this.vision = vision;

    if (SpartanEntryManager.isTuningMode()) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    } else {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    }

    try {
      swerve =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve")).createSwerveDrive();
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
          double translationVal =
              translationLimiter.calculate(
                  MathUtil.applyDeadband(
                      translationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          double strafeVal =
              strafeLimiter.calculate(
                  MathUtil.applyDeadband(
                      strafeSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
          double rotationVal =
              rotationLimiter.calculate(
                  MathUtil.applyDeadband(
                      rotationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));

          drive(
              new Translation2d(translationVal, strafeVal)
                  .times(swerve.swerveController.config.maxSpeed),
              rotationVal * swerve.swerveController.config.maxAngularVelocity,
              true,
              false);
        })
        .withName("TeleopSwerve");
  }

  public void drive(
      Translation2d translationVal, double rotationVal, boolean fieldRelative, boolean openLoop) {
    swerve.drive(translationVal, rotationVal, fieldRelative, openLoop);
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    swerve.setChassisSpeeds(speeds);
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorIdleMode(brake);
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  public void lock() {
    swerve.lockPose();
  }

  public double getYaw() {
    return swerve.getYaw().getDegrees();
  }

  public double getPitch() {
    return swerve.getPitch().getDegrees();
  }

  public void resetOdometry(Pose2d pose) {
    swerve.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerve.getPose();
  }

  public ChassisSpeeds getFieldVelocity() {
    return swerve.getFieldVelocity();
  }

  public Optional<Translation3d> getAcceleration() {
    return swerve.getAccel();
  }

  public void resetToAbsolute() {
    swerve.synchronizeModuleEncoders();
  }

  @Override
  public void periodic() {
    swerve.updateOdometry();

    Optional<EstimatedRobotPose> result = vision.getEstimatedGlobalPose(getPose());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      swerve.addVisionMeasurement(
          camPose.estimatedPose.toPose2d(), camPose.timestampSeconds, true, 0.5);
    }
  }
}
