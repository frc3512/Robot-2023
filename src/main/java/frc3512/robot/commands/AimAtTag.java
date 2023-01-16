package frc3512.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class AimAtTag extends CommandBase {
  private final Swerve s_Swerve;
  private final Vision m_vision;
  private final Timer timer = new Timer();

  public AimAtTag(Swerve swerve, Vision vision) {
    this.s_Swerve = swerve;
    this.m_vision = vision;

    addRequirements(s_Swerve, vision);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() {
    timer.start();
    double rotationSpeed = -s_Swerve.controller.calculate(m_vision.getYaw(), 0);
    s_Swerve.drive(new Translation2d(), rotationSpeed, true, true);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    s_Swerve.drive(new Translation2d(), 0.0, true, true);
  }

  @Override
  public boolean isFinished() {
    return s_Swerve.controller.atSetpoint() || timer.hasElapsed(1.0);
  }
}
