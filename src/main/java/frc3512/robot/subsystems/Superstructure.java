package frc3512.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private final Intake intake;

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

  // Grid enum
  public enum GridSelection {
    LEFT_CONE,
    CUBE_MIDDLE,
    RIGHT_CONE
  }

  private ScoringEnum scoringSelection = ScoringEnum.STOW;
  private Pose2d scoringPose;

  public Superstructure(Swerve swerve, Elevator elevator, Arm arm, Intake intake, LEDs leds) {
    this.swerve = swerve;
    this.elevator = elevator;
    this.arm = arm;
    this.leds = leds;
    this.intake = intake;
    scoringPose = swerve.getPose();

    autos = new Autos(swerve, elevator, arm, this, intake);
  }

  public Command getAuton() {
    return autos.getSelected();
  }

  public Command autoScore() {
    return goToPreset(scoringSelection)
        .andThen(new WaitCommand(0.3))
        .andThen(intake.intakeGamePiece())
        .andThen(new WaitCommand(0.2))
        .andThen(intake.stopIntake())
        .andThen(goToPreset(ScoringEnum.STOW));
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
        elevator.setGoal(elevatorState), new WaitCommand(0.25), arm.setGoal(armState));
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
        .andThen(new WaitCommand(1.0))
        .andThen(leds.goToElementColor());
  }

  public Command prepareToScore(ScoringEnum scoringEnum, GridSelection gridSelection) {
    scoringSelection = scoringEnum;

    if (gridSelection == GridSelection.CUBE_MIDDLE) {
      scoringPose = swerve.getPose();
    } else if (gridSelection == GridSelection.LEFT_CONE) {
      if (scoringEnum == ScoringEnum.SCORE_CONE_L2) {
        scoringPose =
            new Pose2d(
                new Translation2d(swerve.getPose().getX() - 1.0, swerve.getPose().getY() + 1.0),
                swerve.getPose().getRotation());
      } else {
        scoringPose =
            new Pose2d(
                new Translation2d(swerve.getPose().getX(), swerve.getPose().getY() + 1.0),
                swerve.getPose().getRotation());
      }
    } else if (gridSelection == GridSelection.RIGHT_CONE) {
      if (scoringEnum == ScoringEnum.SCORE_CONE_L2) {
        scoringPose =
            new Pose2d(
                new Translation2d(swerve.getPose().getX() - 1.0, swerve.getPose().getY() - 1.0),
                swerve.getPose().getRotation());
      } else {
        scoringPose =
            new Pose2d(
                new Translation2d(swerve.getPose().getX(), swerve.getPose().getY() - 1.0),
                swerve.getPose().getRotation());
      }
    }

    return new DriveToPose(swerve, () -> swerve.getPose(), scoringPose, false)
        .withTimeout(1.0)
        .andThen(leds.selectMode(Selection.READY_TO_SCORE));
  }

  public Command moveRobotLeft() {
    return leds.selectMode(Selection.ADJUSTING)
        .andThen(
            new DriveToPose(
                    swerve,
                    () -> swerve.getPose(),
                    scoringPose.plus(
                        new Transform2d(new Translation2d(0.0, 0.1), Rotation2d.fromDegrees(0.0))),
                    false)
                .withTimeout(1.0)
                .andThen(leds.selectMode(Selection.READY_TO_SCORE)));
  }

  public Command moveRobotRight() {
    return leds.selectMode(Selection.ADJUSTING)
        .andThen(
            new DriveToPose(
                    swerve,
                    () -> swerve.getPose(),
                    scoringPose.plus(
                        new Transform2d(new Translation2d(0.0, -0.1), Rotation2d.fromDegrees(0.0))),
                    false)
                .withTimeout(1.0)
                .andThen(leds.selectMode(Selection.READY_TO_SCORE)));
  }

  public Command moveRobotForward() {
    return leds.selectMode(Selection.ADJUSTING)
        .andThen(
            new DriveToPose(
                    swerve,
                    () -> swerve.getPose(),
                    scoringPose.plus(
                        new Transform2d(new Translation2d(0.1, 0.0), Rotation2d.fromDegrees(0.0))),
                    false)
                .withTimeout(1.0)
                .andThen(leds.selectMode(Selection.READY_TO_SCORE)));
  }

  public Command moveRobotBack() {
    return leds.selectMode(Selection.ADJUSTING)
        .andThen(
            new DriveToPose(
                    swerve,
                    () -> swerve.getPose(),
                    scoringPose.plus(
                        new Transform2d(new Translation2d(-0.1, 0.0), Rotation2d.fromDegrees(0.0))),
                    false)
                .withTimeout(1.0)
                .andThen(leds.selectMode(Selection.READY_TO_SCORE)));
  }
}
