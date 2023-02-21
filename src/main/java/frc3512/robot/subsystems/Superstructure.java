package frc3512.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc3512.robot.auton.Autos;
import frc3512.robot.commands.DriveToPose;

public class Superstructure {
  // Subsystems
  private final Swerve swerve;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;

  // Autons
  private final Autos autos;

  public Superstructure(Swerve swerve, Elevator elevator, Arm arm, Intake intake) {
    this.swerve = swerve;
    this.elevator = elevator;
    this.intake = intake;
    this.arm = arm;

    autos = new Autos(swerve, elevator, intake, arm);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public Command driveToPose(Pose2d pose) {
    return new DriveToPose(swerve, pose);
  }
}
