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

  // Autons
  private final Autos autos;

  // Scoring enum
  public enum ScoringEnum {
    INTAKE,
    STOW,
    CONE_PLAYER_STATION,
    CUBE_PLAYER_STATION,
    SCORE_CUBE_L2,
    SCORE_CUBE_L3,
    SCORE_CONE_L2,
    SCORE_CONE_L3
  }

  public Superstructure(Swerve swerve, Elevator elevator, Arm arm, Intake intake) {
    this.swerve = swerve;
    this.elevator = elevator;
    this.arm = arm;

    autos = new Autos(swerve, elevator, arm, this, intake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public Command goToPreset(ScoringEnum scoringPose) {
    if (scoringPose == ScoringEnum.INTAKE) {
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(2.37, 0.0));
    } else if (scoringPose == ScoringEnum.STOW) {
      return goToScoreSetpointHyrbid(
          new State(0.0, 0.0), new State(Constants.ArmConstants.stowValue, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CUBE_L2) {
      return goToScoreSetpoint(new State(0.0, 0.0), new State(2.25, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CUBE_L3) {
      return goToScoreSetpoint(new State(0.22, 0.0), new State(2.7, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CONE_L2) {
      return goToScoreSetpoint(new State(0.20, 0.0), new State(2.7, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CONE_L3) {
      return goToScoreSetpoint(new State(0.35, 0.0), new State(2.5, 0.0));
    } else if (scoringPose == ScoringEnum.CONE_PLAYER_STATION) {
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(3.41, 0.0));
    } else if (scoringPose == ScoringEnum.CUBE_PLAYER_STATION) {
      return goToScoreSetpointHyrbid(new State(0.0, 0.0), new State(3.49, 0.0));
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

  public Command enableAutoControl() {
    return Commands.runOnce(
        () -> {
          elevator.enable();
          arm.enable();
        },
        arm,
        elevator);
  }

  public Command driveToClosetPose() {
    return new DriveToPose(swerve, findClosestPose());
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
}
