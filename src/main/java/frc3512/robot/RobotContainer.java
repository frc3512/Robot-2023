package frc3512.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.auton.Autos;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class RobotContainer {
  // Robot subsystems
  private Vision m_vision = new Vision();
  private Swerve m_swerve = new Swerve(m_vision);

  // Autos
  private Autos autos = new Autos(m_swerve);

  // Xbox controllers
  private final CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.xboxController1Port);

  // Drive Controls
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public RobotContainer() {
    configureButtonBindings();
    configureAxisActions();
  }

  /** Used for defining button actions. */
  private void configureButtonBindings() {

    /* Driver Buttons */
    driver.x().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));
  }

  /** Used for joystick/xbox axis actions. */
  private void configureAxisActions() {
    m_swerve.setDefaultCommand(
        m_swerve.driveWithJoysticks(
            () -> driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> driver.getRawAxis(rotationAxis)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autos.getAuton();
  }
}
