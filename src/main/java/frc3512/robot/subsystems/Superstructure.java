package frc3512.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc3512.robot.commands.DriveToPose;

public class Superstructure {
  private final Swerve swerve;

  public Superstructure(Swerve swerve) {
    this.swerve = swerve;
  }

  public Command driveToPose(Pose2d pose) {
    return new DriveToPose(swerve, pose);
  }
}
