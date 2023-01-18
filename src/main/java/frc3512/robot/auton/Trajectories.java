package frc3512.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;
import java.util.HashMap;
import java.util.List;

@SuppressWarnings("unused")
public final class Trajectories {

  private final Swerve swerve;
  private final SwerveAutoBuilder autoBuilder;
  private final HashMap<String, Command> eventMap;

  public Trajectories(Swerve swerve) {
    this.swerve = swerve;

    eventMap = new HashMap<>();
    assignMarkerDuties();

    autoBuilder =
        new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            Constants.SwerveConstants.swerveKinematics,
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve);
  }

  public void assignMarkerDuties() {}

  public Command followPath(PathPlannerTrajectory trajectory) {
    return autoBuilder.followPath(trajectory);
  }

  public Command followPathWithEvents(PathPlannerTrajectory trajectory) {
    return autoBuilder.followPathWithEvents(trajectory);
  }

  public Command followPathGroup(List<PathPlannerTrajectory> list) {
    return autoBuilder.followPathGroup(list);
  }

  public Command followPathGroupWithEvents(List<PathPlannerTrajectory> list) {
    return autoBuilder.followPathGroupWithEvents(list);
  }

  public Command createAuton(PathPlannerTrajectory trajectory) {
    return autoBuilder.fullAuto(trajectory);
  }

  public Command createAuton(List<PathPlannerTrajectory> list) {
    return autoBuilder.fullAuto(list);
  }
}
