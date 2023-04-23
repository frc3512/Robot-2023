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
import frc3512.robot.subsystems.LEDs.Selection;

public class Superstructure extends SubsystemBase {
  // Subsystems
  private final Swerve swerve;
  private final Elevator elevator;
  private final Arm arm;
  private final LEDs leds;

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

  public Superstructure(Swerve swerve, Elevator elevator, Arm arm, Intake intake, LEDs leds) {
    this.swerve = swerve;
    this.elevator = elevator;
    this.arm = arm;
    this.leds = leds;

    autos = new Autos(swerve, elevator, arm, this, intake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public Command goToPreset(ScoringEnum scoringPose) {
    if (scoringPose == ScoringEnum.INTAKE) {
      return goToScoreSetpointHyrbid(
          new State(Constants.ScoringConstants.elevatorLeveled, 0.0),
          new State(Constants.ScoringConstants.armIntake, 0.0));
    } else if (scoringPose == ScoringEnum.STOW) {
      return goToScoreSetpointHyrbid(
          new State(Constants.ScoringConstants.elevatorLeveled, 0.0),
          new State(Constants.ArmConstants.stowValue, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CUBE_L2) {
      return goToScoreSetpoint(
          new State(Constants.ScoringConstants.elevatorLeveled, 0.0),
          new State(Constants.ScoringConstants.armCubeL2, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CUBE_L3) {
      return goToScoreSetpoint(
          new State(Constants.ScoringConstants.elevatorCubeL3, 0.0),
          new State(Constants.ScoringConstants.armCubeL3, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CONE_L2) {
      return goToScoreSetpoint(
          new State(Constants.ScoringConstants.elevatorConeL2, 0.0),
          new State(Constants.ScoringConstants.armConeL2, 0.0));
    } else if (scoringPose == ScoringEnum.SCORE_CONE_L3) {
      return goToScoreSetpoint(
          new State(Constants.ScoringConstants.elevatorConeL3, 0.0),
          new State(Constants.ScoringConstants.armConeL3, 0.0));
    } else if (scoringPose == ScoringEnum.CONE_PLAYER_STATION) {
      return goToScoreSetpointHyrbid(
          new State(Constants.ScoringConstants.elevatorLeveled, 0.0),
          new State(Constants.ScoringConstants.armConeHP, 0.0));
    } else if (scoringPose == ScoringEnum.CUBE_PLAYER_STATION) {
      return goToScoreSetpointHyrbid(
          new State(Constants.ScoringConstants.elevatorLeveled, 0.0),
          new State(Constants.ScoringConstants.armCubeHP, 0.0));
    } else {
      // Default choice: Stowed
      return goToScoreSetpointHyrbid(
          new State(Constants.ScoringConstants.elevatorLeveled, 0.0),
          new State(Constants.ArmConstants.stowValue, 0.0));
    }
  }

  public Command goToScoreSetpoint(
      TrapezoidProfile.State elevatorState, TrapezoidProfile.State armState) {
    return Commands.sequence(
            elevator.setGoal(elevatorState), new WaitCommand(0.25), arm.setGoal(armState))
        .andThen(new WaitCommand(1.0))
        .andThen(leds.selectMode(Selection.READY_TO_SCORE));
  }

  public Command goToScoreSetpointHyrbid(
      TrapezoidProfile.State elevatorState, TrapezoidProfile.State armState) {
    return Commands.sequence(arm.setGoal(armState), elevator.setGoal(elevatorState))
        .andThen(leds.goToElementColor());
  }

  public Command enableManualControl() {
    return Commands.runOnce(
            () -> {
              elevator.disable();
              arm.disable();
            },
            arm,
            elevator)
        .andThen(leds.selectMode(Selection.MANUAL_MODE))
        .andThen(new WaitCommand(0.5))
        .andThen(leds.goToElementColor());
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
