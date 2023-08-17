package frc3512.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.Elevator;
import frc3512.robot.subsystems.Intake;
import frc3512.robot.subsystems.LEDs;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Superstructure.ScoringEnum;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class Robot2023 {
  // Robot subsystems
  private Vision vision = new Vision();
  private Swerve swerve = new Swerve(vision);
  private Elevator elevator = new Elevator();
  private Arm arm = new Arm();
  private Intake intake = new Intake();
  private LEDs leds = new LEDs();
  private Superstructure superstructure = new Superstructure(swerve, elevator, arm, intake, leds);

  // Driver Control
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Joysticks
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.driverControllerPort);
  private final CommandJoystick appendage =
      new CommandJoystick(Constants.OperatorConstants.appendageControllerPort);

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }

  /** Used for defining button actions. */
  public void configureButtonBindings() {

    driver.x().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    driver.leftBumper().onTrue(leds.switchColor());
    driver.rightBumper().whileTrue(superstructure.prepareToScore());

    appendage.axisLessThan(Joystick.AxisType.kX.value, -0.5).whileTrue(intake.intakeGamePiece());
    appendage.axisGreaterThan(Joystick.AxisType.kX.value, 0.5).onTrue(intake.outtakeGamePiece());
    appendage
        .axisLessThan(Joystick.AxisType.kY.value, -0.5)
        .whileTrue(superstructure.goToPreset(ScoringEnum.STOW));
    appendage
        .axisGreaterThan(Joystick.AxisType.kY.value, 0.5)
        .onTrue(superstructure.goToPreset(ScoringEnum.INTAKE));
    appendage.button(1).onTrue(superstructure.goToPreset(ScoringEnum.SCORE_CONE_L3));
    appendage.button(2).onTrue(superstructure.goToPreset(ScoringEnum.SCORE_CUBE_L3));
    appendage.button(3).onTrue(superstructure.goToPreset(ScoringEnum.SCORE_CONE_L3));
    appendage.button(4).onTrue(superstructure.goToPreset(ScoringEnum.SCORE_CONE_L2));
    appendage.button(5).onTrue(superstructure.goToPreset(ScoringEnum.SCORE_CUBE_L2));
    appendage.button(6).onTrue(superstructure.goToPreset(ScoringEnum.SCORE_CONE_L2));
    appendage.button(7).onTrue(intake.halfOuttakeGamePiece());
    appendage.button(9).onTrue(superstructure.enableManualControl());
    appendage.button(9).onFalse(superstructure.disableManualControl());
    appendage.button(10).onTrue(intake.stopIntake());
    appendage.button(11).onTrue(superstructure.goToPreset(ScoringEnum.CONE_PLAYER_STATION));
    appendage.button(12).onTrue(superstructure.goToPreset(ScoringEnum.CUBE_PLAYER_STATION));
  }

  /** Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.drive(
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));

    /**
     * elevator.setDefaultCommand( elevator.runElevator(() ->
     * MathUtil.applyDeadband(-appendage.getRawAxis(1), 0.01)));
     *
     * <p>arm.setDefaultCommand(arm.runArm(() -> appendage.getHID().getPOV()));
     */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return superstructure.getAuton();
  }
}
