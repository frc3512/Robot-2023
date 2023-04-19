package frc3512.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;
import java.util.function.Supplier;

// Modified version of 6328's DriveToPose Command
// https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/commands/DriveToPose.java
public class DriveToPose extends CommandBase {
  private final Swerve swerve;
  private final Supplier<Pose2d> poseSupplier;

  private boolean running = false;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 3.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 3.0));
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private Translation2d lastSetpointTranslation;

  public DriveToPose(Swerve swerve, Pose2d pose) {
    this(swerve, () -> pose);
  }

  public DriveToPose(Swerve swerve, Supplier<Pose2d> poseSupplier) {
    this.swerve = swerve;
    this.poseSupplier = poseSupplier;
    addRequirements(swerve);
    driveController.setTolerance(0.01);
    thetaController.setTolerance(Units.degreesToRadians(0.9));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public DriveToPose(Swerve swerve) {
    this(swerve, () -> new Pose2d());
  }

  @Override
  public void initialize() {
    var currentPose = swerve.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                    swerve.getFieldVelocity().vxMetersPerSecond,
                    swerve.getFieldVelocity().vyMetersPerSecond)
                .rotateBy(
                    poseSupplier
                        .get()
                        .getTranslation()
                        .minus(swerve.getPose().getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(currentPose.getRotation().getRadians(), swerve.getYaw());
    lastSetpointTranslation = swerve.getPose().getTranslation();
  }

  @Override
  public void execute() {
    running = true;

    var currentPose = swerve.getPose();
    var targetPose = poseSupplier.get();

    double currentDistance =
        currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity + driveController.calculate(driveErrorAbs, 0.0);
    if (driveController.atGoal()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    double thetaVelocity =
        thetaController.getSetpoint().velocity
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
    if (thetaController.atGoal()) thetaVelocity = 0.0;

    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
            .transformBy(translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();
    swerve.setChassisSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    running = false;
    swerve.setChassisSpeeds(new ChassisSpeeds());
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  public boolean isRunning() {
    return running;
  }

  private static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }
}
