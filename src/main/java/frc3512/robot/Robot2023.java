package frc3512.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.subsystems.Arm;
import frc3512.robot.subsystems.Elevator;
import frc3512.robot.subsystems.Intake;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class Robot2023 {
  // Robot subsystems
  private Vision vision = new Vision();
  private Swerve swerve = new Swerve(vision);
  private Elevator elevator = new Elevator();
  private Arm arm = new Arm();
  private Intake intake = new Intake();
  private Superstructure superstructure = new Superstructure(swerve, elevator, arm, intake);

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

    appendage.button(1).whileTrue(intake.stopIntake());
    appendage.button(5).whileTrue(intake.intakeGamePiece());
    appendage.button(6).whileTrue(intake.outtakeGamePiece());

    appendage.button(7).onTrue(elevator.setGoal(new State(0.1, 0.0)));
    appendage.button(9).onTrue(elevator.setGoal(new State(0.0, 0.0)));
  }

  /** Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    swerve.setDefaultCommand(
        swerve.drive(
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));
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
