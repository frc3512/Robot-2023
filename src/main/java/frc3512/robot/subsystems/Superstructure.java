package frc3512.robot.subsystems;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc3512.robot.Constants;
import frc3512.robot.auton.Autos;

public class Superstructure extends SubsystemBase {
  // Subsystems
  private final Elevator elevator;
  private final Arm arm;

  // Autons
  private final Autos autos;

  // Scoring enum
  public enum ScoringEnum {
    INTAKE,
    STOW,
    SINGLE_PLAYER_STATION,
    DOUBLE_PLAYER_STATION,
    SCORE_CUBE_L2,
    SCORE_CUBE_L3,
    SCORE_CONE_L2,
    SCORE_CONE_L3
  }

  public Superstructure(Swerve swerve, Elevator elevator, Arm arm, Intake intake) {
    this.elevator = elevator;
    this.arm = arm;

    autos = new Autos(swerve, elevator, arm, this, intake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public Command goToPreset(ScoringEnum scoringPose) {
    if (scoringPose == ScoringEnum.INTAKE) {
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(1.33, 0.0));
    } else if (scoringPose == ScoringEnum.STOW) {
      return goToScoreSetpointHyrbid(
          new State(0.0, 0.0), new State(Constants.ArmConstants.stowValue, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CUBE_L2) {
      return goToScoreSetpoint(new State(0.0, 0.0), new State(2.25, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CUBE_L3) {
      return goToScoreSetpoint(new State(0.26, 0.0), new State(1.75, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CONE_L2) {
      return goToScoreSetpoint(new State(0.18, 0.0), new State(1.75, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CONE_L3) {
      return goToScoreSetpoint(new State(0.35, 0.0), new State(1.43, 0.0));
    } else if (scoringPose == ScoringEnum.SINGLE_PLAYER_STATION) {
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(2.30, 0.0));
    } else if (scoringPose == ScoringEnum.DOUBLE_PLAYER_STATION) {
      return goToScoreSetpoint(new State(0.35, 0.0), new State(1.03, 0.0));
    } else {
      // Default choice: Stowed
      return goToScoreSetpointHyrbid(
          new State(0.0, 0.0), new State(Constants.ArmConstants.stowValue, 0.0));
    }
  }

  public Command goToScoreSetpoint(
      TrapezoidProfile.State elevatorState, TrapezoidProfile.State armState) {
    return Commands.sequence(
        elevator.setGoal(elevatorState), new WaitCommand(0.25), arm.setGoal(armState));
  }

  public Command goToScoreSetpointHyrbid(
      TrapezoidProfile.State elevatorState, TrapezoidProfile.State armState) {
    return Commands.sequence(arm.setGoal(armState), elevator.setGoal(elevatorState));
  }

  public Command enableManualControl() {
    return Commands.runOnce(
        () -> {
          elevator.disable();
          arm.disable();
        },
        arm,
        elevator);
  }
}
