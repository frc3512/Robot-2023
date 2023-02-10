package frc3512.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class AprilTagAim extends CommandBase {
  private final Swerve swerve;
  private final Vision vision;
  private final Timer timer = new Timer();

  public PIDController rotationController = new PIDController(0.05, 0, 0);

  public AprilTagAim(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;

    addRequirements(swerve, vision);
  }

  @Override
  public void initialize() {
    rotationController.setTolerance(4.5);
    timer.reset();
  }

  @Override
  public void execute() {
    if (vision.hasTarget()) {
      timer.start();
      double rotationSpeed = rotationController.calculate(vision.getBestMeasurement().yaw, 0);
      swerve.drive(new Translation2d(), rotationSpeed, false, false);
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
    return rotationController.atSetpoint() || timer.hasElapsed(1.0);
  }
}
