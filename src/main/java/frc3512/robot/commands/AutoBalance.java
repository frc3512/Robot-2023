package frc3512.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
  private Swerve swerve;
  private PIDController controller;

  public AutoBalance(Swerve swerve) {
    this.swerve = swerve;
    controller = new PIDController(0.5, 0.0, 0.0);
    controller.setTolerance(1);
    controller.setSetpoint(0.0);
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("At Setpoint", controller.atSetpoint());

    double translationVal = MathUtil.clamp(controller.calculate(swerve.getPitch()), -0.5, 0.5);
    swerve.drive(new Translation2d(translationVal, 0.0), 0.0, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.lock();
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}
