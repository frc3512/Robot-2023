package frc3512.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.wpilibj2.command.Command;
import frc3512.lib.swerve.SpartanSwerveAutonBuilder;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;
import java.util.HashMap;

public final class Autos {

  private SpartanSwerveAutonBuilder autonBuilder;
  private PathConstraints constraints;
  private HashMap<String, Command> eventMap;

  public Autos(Swerve swerve) {
    eventMap = new HashMap<>();
    configureMarkers();

    constraints = new PathConstraints(1.0, 4.0);

    autonBuilder =
        new SpartanSwerveAutonBuilder(
            Constants.SwerveConstants.swerveKinematics,
            new PIDConstants(0.5, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            swerve::getPose,
            swerve::resetOdometry,
            swerve::setModuleStates,
            swerve::setYaw,
            eventMap,
            swerve);
  }

  private void configureMarkers() {}

  public Command score2FarZone() {
    return autonBuilder.fullAuto(PathPlanner.loadPath("Score 2 Far Zone", constraints));
  }
}
