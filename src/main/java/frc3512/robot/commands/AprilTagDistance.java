package frc3512.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class AprilTagDistance extends CommandBase {
  private final Swerve swerve;
  private final Vision vision;
  private final Timer timer = new Timer();
  private final double rangeGoal;

  public PIDController translationController = new PIDController(0.9, 0, 0);
  public PIDController strafeController = new PIDController(0.5, 0, 0);

  public AprilTagDistance(Swerve swerve, Vision vision, double desiredRangeMeters) {
    this.swerve = swerve;
    this.vision = vision;
    this.rangeGoal = desiredRangeMeters;

    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    translationController.setTolerance(0.1);
    strafeController.setTolerance(0.5);
    timer.reset();
  }

  @Override
  public void execute() {
    if (vision.hasTarget()) {
      timer.start();
      double translationSpeed =
          -translationController.calculate(vision.getBestMeasurement().distance, rangeGoal);
      double strafeSpeed = strafeController.calculate(vision.getBestMeasurement().yaw, 0);
      swerve.drive(new Translation2d(translationSpeed, strafeSpeed), 0.0, false, false);
    } else {
      swerve.drive(new Translation2d(), 0.0, false, false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    swerve.drive(new Translation2d(), 0.0, true, true);
  }

  @Override
  public boolean isFinished() {
    return (translationController.atSetpoint() && strafeController.atSetpoint())
        || timer.hasElapsed(1.0);
  }
}
