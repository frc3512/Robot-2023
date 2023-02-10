package frc3512.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc3512.robot.commands.AprilTagAim;
import frc3512.robot.commands.AprilTagDistance;

public class Superstructure {
  private final Swerve swerve;
  private final Vision vision;

  public Superstructure(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;
  }

  public Command faceAprilTag(double distanceMeters) {
    return Commands.sequence(
        new AprilTagDistance(swerve, vision, distanceMeters), new AprilTagAim(swerve, vision));
  }
}
