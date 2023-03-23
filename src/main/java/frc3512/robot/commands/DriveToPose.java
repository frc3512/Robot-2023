package frc3512.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;

public class DriveToPose extends CommandBase {

  private final Swerve swerve;
  private Pose2d targetPose;
  private Pose2d currentPose;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 3.0));
  private final ProfiledPIDController yController =
      new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 3.0));
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(2.0, 0.0, 0.0, new TrapezoidProfile.Constraints(4.0, 3.0));

  public DriveToPose(Swerve swerve) {
    this.swerve = swerve;

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Units.degreesToRadians(0.5));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    currentPose = swerve.getPose();
    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());
    thetaController.reset(currentPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    currentPose = swerve.getPose();
    targetPose = findClosestPose();

    double xvelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    double yvelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    double thetaVelocity =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    if (atGoal()) {
      xvelocity = 0.0;
      yvelocity = 0.0;
      thetaVelocity = 0.0;
    }

    swerve.setChassisSpeeds(new ChassisSpeeds(xvelocity, yvelocity, thetaVelocity));

    SmartDashboard.putNumberArray("Current Pose", new double[] {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()});
    SmartDashboard.putNumberArray("Target Pose", new double[] {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()});
    SmartDashboard.putBoolean("X At Goal", xController.atGoal());
    SmartDashboard.putBoolean("Y At Goal", yController.atGoal());
    SmartDashboard.putBoolean("Theta At Goal", thetaController.atGoal());
  }

  public boolean atGoal() {
    return (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
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
