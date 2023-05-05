package frc3512.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;
import java.util.function.Supplier;

/** Command to drive to a pose. */
public class DriveToPose extends CommandBase {

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final Swerve drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final boolean useAllianceColor;

  public DriveToPose(
      Swerve drivetrainSubsystem, Supplier<Pose2d> poseProvider, boolean useAllianceColor) {
    this(
        drivetrainSubsystem,
        poseProvider,
        new TrapezoidProfile.Constraints(4.0, 4.0),
        new TrapezoidProfile.Constraints(4.0, 4.0),
        useAllianceColor);
  }

  public DriveToPose(
      Swerve drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      TrapezoidProfile.Constraints xyConstraints,
      TrapezoidProfile.Constraints omegaConstraints,
      boolean useAllianceColor) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.useAllianceColor = useAllianceColor;

    xController = new ProfiledPIDController(1.5, 0.0, 0.0, xyConstraints);
    yController = new ProfiledPIDController(1.5, 0.0, 0.0, xyConstraints);
    thetaController = new ProfiledPIDController(1.5, 0.0, 0.0, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    resetPIDControllers();
    var pose = findClosestPose();
    if (useAllianceColor && DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      Translation2d transformedTranslation = new Translation2d(pose.getX(), 8.0137 - pose.getY());
      Rotation2d transformedHeading = pose.getRotation().times(-1);
      pose = new Pose2d(transformedTranslation, transformedHeading);
    }
    thetaController.setGoal(pose.getRotation().getRadians());
    xController.setGoal(pose.getX());
    yController.setGoal(pose.getY());
  }

  private Pose2d findClosestPose() {
    Pose2d closestPose = Constants.ScoringConstants.redScoringPositions.get(0);
    for (Pose2d pose : Constants.ScoringConstants.redScoringPositions) {
      if (closestPose.relativeTo(drivetrainSubsystem.getPose()).getTranslation().getNorm()
          > pose.relativeTo(drivetrainSubsystem.getPose()).getTranslation().getNorm()) {
        closestPose = pose;
      }
    }
    return closestPose;
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.setChassisSpeeds(new ChassisSpeeds());
  }
}
