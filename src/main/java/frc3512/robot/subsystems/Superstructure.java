package frc3512.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc3512.robot.Constants;
import frc3512.robot.auton.Autos;
import frc3512.robot.commands.DriveToPose;

public class Superstructure extends SubsystemBase {
  // Subsystems
  private final Swerve swerve;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;

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
    this.swerve = swerve;
    this.elevator = elevator;
    this.intake = intake;
    this.arm = arm;

    autos = new Autos(swerve, this, intake);
  }

  public Command getAuton() {
    return autos.getSelected();
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

  public Command goToScoreSetpoint(
      TrapezoidProfile.State elevatorState, TrapezoidProfile.State armState) {
    return Commands.sequence(
        elevator.setGoal(elevatorState), new WaitCommand(0.25), arm.setGoal(armState));
  }

  public Command goToScoreSetpointHyrbid(
      TrapezoidProfile.State elevatorState, TrapezoidProfile.State armState) {
    return Commands.sequence(arm.setGoal(armState), elevator.setGoal(elevatorState));
  }

  public Command goToPreset(ScoringEnum scoringPose) {
    if (scoringPose == ScoringEnum.INTAKE) {
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(1.33, 0.0));
    } else if (scoringPose == ScoringEnum.STOW) {
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(2.70, 0.0));
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
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(2.70, 0.0));
    }
  }

  public Command autoScore(ScoringEnum scoringPose) {
    return goToPreset(scoringPose)
        .andThen(new WaitCommand(1.5))
        .andThen(intake.intakeGamePiece().withTimeout(1.0))
        .andThen(goToPreset(ScoringEnum.STOW))
        .andThen(intake.stopIntake());
  }

  private Pose2d findClosestPose() {
    Pose2d closestPose = Constants.FieldConstants.scoringPositions.get(0);
    for (Pose2d pose : Constants.FieldConstants.scoringPositions) {
      if (closestPose.relativeTo(swerve.getPose()).getTranslation().getNorm()
          > pose.relativeTo(swerve.getPose()).getTranslation().getNorm()) {
        closestPose = pose;
      }
    }
    return new Pose2d();
  }

  public Command driveToClosetPose() {
    return new DriveToPose(swerve, findClosestPose());
  }
}
