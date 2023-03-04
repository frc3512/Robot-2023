package frc3512.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.Elevator;
import frc3512.robot.subsystems.Intake;

public class AutoDrop {
  private Intake intake = new Intake();
  private Arm arm = new Arm();
  private Elevator elevator = new Elevator();

  public Command automaticIntake(int gamePiece) {
    double speed;
    if (gamePiece == 0) {
      speed = 0.65;
    } else if (gamePiece == 1) {
      speed = -0.65;
    }
    return (
        // arm to the horizontal potition
        arm.setGoal(0.0, 0.0)
      ).andThen(
        // elevator to the ground position
        elevator.setGoal(new TrapezoidProfile.State(0.0, 0.0))
      ).andThen(
        // intake the game piece, 0 is cube, 1 is cone
        intake.customIntake(speed)
      );
  }

  public void automaticScoring(double height, int station, int gamePiece) {
    if (station == 0) {
      // go to the right station
    } else if (station == 1) {
      // go to the middle station
    } else if (station == 2) {
      // go to the left station
    }

    // drive elevator to the height

    if (gamePiece == 0) {
      intake.outtakeCube();
    } else if (gamePiece == 1) {
      intake.outtakeCone();
    }
  }

  public Command scoreGamePiece(double level) {
    return Commands.sequence(elevator.setGoal(new TrapezoidProfile.State(level, 0.0)), arm.setGoal());
  }
}
