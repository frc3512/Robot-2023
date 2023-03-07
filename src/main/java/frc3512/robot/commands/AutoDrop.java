package frc3512.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc3512.robot.commands.DriveToPose;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.Elevator;
import frc3512.robot.subsystems.Intake;
import frc3512.robot.subsystems.Superstructure;

public class AutoDrop {
  private Intake intake = new Intake();
  private Arm arm = new Arm();
  private Elevator elevator = new Elevator();
  
  public Command automaticGroundIntake() {
    return (
        // arm to the horizontal potition
        arm.setGoal()
      ).andThen(
        // elevator to the ground position
        elevator.setGoal(new TrapezoidProfile.State(0.0, 0.0))
      ).andThen(
        intake.intakeGamePiece()
      );
  }

  public Command automaticHPIntake() {
    return (
        // arm to the horizontal potition
        arm.setGoal(new TrapezoidProfile.State(0.08, 0.0)
      ).andThen(
        // elevator to the ground position
        elevator.setGoal(new TrapezoidProfile.State(0.35, 0.0))
      ).andThen(
        intake.intakeGamePiece()
      ));
  }

  public void automaticScoring(double height, int station, int gamePiece) {
    if (station == 0) {
      // go to the right station DriveToPose
    } else if (station == 1) {
      // go to the middle station
    } else if (station == 2) {
      // go to the left station
    }

    // drive elevator to the height
    elevator.setGoal((new TrapezoidProfile.State(height, 0.0)));

    intake.outtakeGamePiece();
  }

  public Command setupScoring(double elevatorHeight, double armHeight) {
    return Commands.sequence(
      elevator.setGoal(new TrapezoidProfile.State(elevatorHeight, 0.0)), 
      arm.setGoal((new TrapezoidProfile.State(armHeight, 0.0))
      );
  }
}
