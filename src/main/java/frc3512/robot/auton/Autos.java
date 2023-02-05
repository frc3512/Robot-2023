package frc3512.robot.auton;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;

public final class Autos {

  private final Swerve swerve;
  private final SendableChooser<Command> autonChooser;

  public Autos(Swerve swerve) {
    this.swerve = swerve;

    autonChooser = new SendableChooser<Command>();
    autonChooser.setDefaultOption("No-op", new InstantCommand());

    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  public Command getSelected() {
    return autonChooser.getSelected();
  }

  public Command score2FarZone() {
    return swerve.followTrajectory(
        PathPlanner.loadPath("Score 2 Far Zone", Constants.AutonConstants.constraints), true);
  }
}
