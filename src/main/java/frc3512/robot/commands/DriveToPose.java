package frc3512.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;

public class DriveToPose extends CommandBase {

  private final Swerve swerve;
  private Pose2d currentPose;
  private ChassisSpeeds currentVelocity;
  private Pose2d desiredPose;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(0.6, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 3.0));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(0.6, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 3.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(2.2, 0.0, 0.0, new TrapezoidProfile.Constraints(3.0, 3.0));

  public DriveToPose(Swerve swerve) {
    this.swerve = swerve;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    currentPose = swerve.getPose();
    currentVelocity = swerve.getFieldVelocity();

    xController.reset(currentPose.getX(), currentVelocity.vxMetersPerSecond);
    yController.reset(currentPose.getY(), currentVelocity.vyMetersPerSecond);
    thetaController.reset(
        currentPose.getRotation().getRadians(), currentVelocity.omegaRadiansPerSecond);

    desiredPose = findClosestPose();
  }

  @Override
  public void execute() {
    double x = xController.calculate(currentPose.getX(), desiredPose.getX());
    double y = yController.calculate(currentPose.getY(), desiredPose.getY());
    double theta =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredPose.getRotation().getRadians());
    ChassisSpeeds speeds = new ChassisSpeeds(x, y, theta);

    swerve.setChassisSpeeds(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(() -> 0.0, () -> 0.0, () -> 0.0);
  }

  private Pose2d findClosestPose() {
    Pose2d closestPose = Constants.FieldConstants.scoringPositions.get(0);
    for (Pose2d pose : Constants.FieldConstants.scoringPositions) {
      if (closestPose.relativeTo(currentPose).getTranslation().getNorm()
          > pose.relativeTo(currentPose).getTranslation().getNorm()) {
        closestPose = pose;
      }
    }
    return closestPose;
  }
}
