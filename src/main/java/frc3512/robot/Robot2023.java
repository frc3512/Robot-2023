package frc3512.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.auton.Autos;
import frc3512.robot.subsystems.Superstructure;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;

public class Robot2023 {
  // Robot subsystems
  private Vision m_vision = new Vision();
  private Swerve m_swerve = new Swerve(m_vision);
  private Superstructure m_superstructure = new Superstructure(m_swerve);

  // Autons
  private final Autos autos = new Autos(m_swerve);

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
    m_swerve.setMotorBrake(brake);
  }

  /** Used for defining button actions. */
  public void configureButtonBindings() {

    /* Driver Buttons */
    driver.x().onTrue(new InstantCommand(() -> m_swerve.zeroGyro()));

    /* Testing Buttons (temp) */
    driver
        .y()
        .whileTrue(
            m_superstructure.driveToPose(
                new Pose2d(new Translation2d(14.54, 5.06), Rotation2d.fromDegrees(0.0))));
  }

  /** Used for joystick/xbox axis actions. */
  public void configureAxisActions() {
    m_swerve.setDefaultCommand(
        m_swerve.drive(
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
    return autos.getSelected();
  }
}
