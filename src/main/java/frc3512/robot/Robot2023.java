package frc3512.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.auton.Autos;
import frc3512.robot.commands.TeleopSwerve;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class Robot2023 {
  // Robot subsystems
  private Vision m_vision = new Vision();
  private Swerve m_swerve = new Swerve(m_vision);

  // Auton Chooser
  private final SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  // Autons
  private final Autos autos = new Autos(m_swerve);

  // Driver Control
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Xbox controllers
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.xboxController1Port);

  /** Used for defining button actions. */
  public void configureButtonBindings() {

    /* Driver Buttons */
    driver.x().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
  }

  /** Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    m_swerve.setDefaultCommand(
        new TeleopSwerve(
            m_swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)));
  }

  /** Used for registering autons. */
  public void registerAutons() {
    autonChooser.setDefaultOption("No-op", new InstantCommand());
    autonChooser.setDefaultOption("Score 2 Far Zone", autos.score2FarZone());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
