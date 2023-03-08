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
import frc3512.robot.commands.AutoBalance;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.Elevator;
import frc3512.robot.subsystems.Intake;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Superstructure.IntakeGamePiece;
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
    autonChooser.addOption("Score 1, Mobility", score1Mobility());
    autonChooser.addOption("Score 1, Balance", score1Balance());

    // autonChooser.addOption("Score 2 Money Zone", score2MoneyZone());
    // autonChooser.addOption("Score 3 Money Zone", score3MoneyZone());
    // autonChooser.addOption("Score 2 Far Zone", score2FarZone());
    // autonChooser.addOption("Score 3 Far Zone", score3FarZone());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  private void setMarkers() {
    eventMap.put("Wait a Second", new WaitCommand(1.5));
    eventMap.put("Stop Intake", intake.stopIntake());
    eventMap.put("Lock Swerve", new InstantCommand(() -> swerve.lock()));
    eventMap.put("Intake", intake.outtakeGamePiece().withTimeout(1.0));
    eventMap.put("Outtake", intake.intakeGamePiece().withTimeout(1.0));
    eventMap.put("Go to Intake Position", superstructure.goToPreset(ScoringEnum.INTAKE));
    eventMap.put("Stow", superstructure.goToPreset(ScoringEnum.STOW));
    eventMap.put("Score Cone L2", superstructure.goToPreset(ScoringEnum.SCORE_CONE_L2));
    eventMap.put("Score Cone L3", superstructure.goToPreset(ScoringEnum.SCORE_CONE_L3));
    eventMap.put("Score Cube L2", superstructure.goToPreset(ScoringEnum.SCORE_CUBE_L2));
    eventMap.put("Score Cube L3", superstructure.goToPreset(ScoringEnum.SCORE_CUBE_L3));
    eventMap.put("Spit Out Cone", superstructure.spitOutGamePiece(IntakeGamePiece.CONE));
    eventMap.put("Spit Out Cube", superstructure.spitOutGamePiece(IntakeGamePiece.CUBE));
    eventMap.put("Auto Balance", new AutoBalance(swerve));
    eventMap.put("Reset Gyro", new InstantCommand(() -> swerve.zeroGyro()));
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  public Command score1Mobility() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Ventura Mobility", Constants.AutonConstants.constraints));
  }

  public Command score1Balance() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Ventura Balance", Constants.AutonConstants.constraints));
  }

  public Command score2MoneyZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Score 2 Money Zone", Constants.AutonConstants.constraints));
  }

  public Command score3MoneyZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPath("Score 3 Money Zone", Constants.AutonConstants.constraints));
  }

  public Command score2FarZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup("Score 2 Far Zone", Constants.AutonConstants.constraints));
  }

  public Command score3FarZone() {
    return autonBuilder.fullAuto(
        PathPlanner.loadPathGroup("Score 3 Far Zone", Constants.AutonConstants.constraints));
  }
}
