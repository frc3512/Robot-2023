package frc3512.robot.auton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc3512.robot.subsystems.Swerve;

@SuppressWarnings("unused")
public class Autos {
  private final Swerve swerve;

  private final SendableChooser<Command> m_autonChooser;
  private final Trajectories trajectoryFollower;

  public Autos(Swerve swerve) {
    this.swerve = swerve;
    m_autonChooser = new SendableChooser<Command>();
    trajectoryFollower = new Trajectories(swerve);
    m_autonChooser.setDefaultOption("No-op", new InstantCommand());

    SmartDashboard.putData("Auton Chooser", m_autonChooser);
  }

  public Command getAuton() {
    return m_autonChooser.getSelected();
  }
}
