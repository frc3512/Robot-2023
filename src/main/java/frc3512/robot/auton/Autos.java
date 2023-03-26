package frc3512.robot.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.Elevator;
import frc3512.robot.subsystems.Intake;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Superstructure.ScoringEnum;
import frc3512.robot.subsystems.Swerve;
import java.util.HashMap;

@SuppressWarnings("unused")
public final class Autos {

  private final Swerve swerve;
  private final Superstructure superstructure;
  private final Elevator elevator;
  private final Arm arm;
  private final Intake intake;
  private final SendableChooser<Command> autonChooser;
  private final HashMap<String, Command> eventMap;
  private final SwerveAutoBuilder autonBuilder;

  public Autos(
      Swerve swerve, Elevator elevator, Arm arm, Superstructure superstructure, Intake intake) {
    this.swerve = swerve;
    this.elevator = elevator;
    this.arm = arm;
    this.superstructure = superstructure;
    this.intake = intake;

    eventMap = new HashMap<>();
    setMarkers();

    autonBuilder =
        new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            new PIDConstants(Constants.AutonConstants.xyControllerP, 0.0, 0.0),
            new PIDConstants(Constants.AutonConstants.thetaControllerP, 0.0, 0.0),
            swerve::setChassisSpeeds,
            eventMap,
            true,
            swerve);

    autonChooser = new SendableChooser<Command>();
    autonChooser.setDefaultOption("No-op", new InstantCommand());
    autonChooser.addOption("Score 1", scoreOne());
    autonChooser.addOption("Score 1, Mobility", score1Mobility());
    autonChooser.addOption("Score 1, Mid Balance", score1Balance());
    autonChooser.addOption("Score 2, No-Cable Mobility", score2NoCable());
    autonChooser.addOption("Score 2, No-Cable Balance", score2NoCableBalance());
    autonChooser.addOption("Score 3, No-Cable Mobility", score3NoCableBalance());
    //autonChooser.addOption("Cable 2 Mobility", cable2Mobility());
    //autonChooser.addOption("Cable 2 Balance", cable2Balance());
    //autonChooser.addOption("No-Cable 2 Mobility", noCable2Mobility());
    //autonChooser.addOption("No-Cable 2 Balance", noCable2Balance());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  private void setMarkers() {
    eventMap.put("Wait a Second", new WaitCommand(1.0));
    eventMap.put("Stop Intake", intake.stopIntake());
    eventMap.put("Intake", intake.outtakeGamePiece().withTimeout(1.0));
    eventMap.put("Outtake", intake.intakeGamePiece().withTimeout(0.5));
    eventMap.put("Intake Position", superstructure.goToPreset(ScoringEnum.INTAKE));
    eventMap.put("Stow", superstructure.goToPreset(ScoringEnum.STOW));
    eventMap.put(
        "Stow and Stop Intake",
        superstructure.goToPreset(ScoringEnum.STOW).andThen(intake.stopIntake()));
    eventMap.put("Score Cone L2", superstructure.goToPreset(ScoringEnum.SCORE_CONE_L2));
    eventMap.put("Score Cone L3", superstructure.goToPreset(ScoringEnum.SCORE_CONE_L3));
    eventMap.put("Score Cube L2", superstructure.goToPreset(ScoringEnum.SCORE_CUBE_L2));
    eventMap.put("Score Cube L3", superstructure.goToPreset(ScoringEnum.SCORE_CUBE_L3));
    eventMap.put("Lock Swerve", new InstantCommand(() -> swerve.lock()));
    eventMap.put("Reset Gyro", new InstantCommand(() -> swerve.zeroGyro()));
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  public Command scoreOne() {
    return superstructure
        .goToPreset(ScoringEnum.SCORE_CONE_L3)
        .andThen(new WaitCommand(1.5))
        .andThen(intake.intakeGamePiece().withTimeout(1.0))
        .andThen(superstructure.goToPreset(ScoringEnum.STOW))
        .andThen(intake.stopIntake());
  }

  public Command score1Mobility() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Score 1 Mobility", Constants.AutonConstants.constraints));
  }

  public Command score1Balance() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Score 1 Mid Balance", Constants.AutonConstants.constraints));
  }

  public Command score2NoCable() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            "Score 2 No-Cable Mobility", Constants.AutonConstants.constraints));
  }

  public Command score2NoCableBalance() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            "Score 2 No-Cable Balance", Constants.AutonConstants.constraints));
  }

  public Command score3NoCableBalance() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            "Score 3 No-Cable Mobility", Constants.AutonConstants.constraints));
  }

  public Command cable2Mobility() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            "Cable 2 Mobility", Constants.AutonConstants.constraints));
  }

  public Command cable2Balance() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            "Cable 2 Balance + grab", Constants.AutonConstants.constraints));
  }

  public Command noCable2Mobility() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            "No-Cable 2 Mobility", Constants.AutonConstants.constraints));
  }

  public Command noCable2Balance() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup(
            "No-Cable 2 Balance + grab", Constants.AutonConstants.constraints));
  }
}
