package frc3512.robot.auton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;

public class TestAuton extends SequentialCommandGroup {
  public TestAuton(Swerve swerve) {
    PathConstraints constraints = new PathConstraints(1.0, 4.0);

    PathPlannerTrajectory driveBack = PathPlanner.loadPath("Score 3 Far Zone", constraints);

    PPSwerveControllerCommand driveBackCommand =
        new PPSwerveControllerCommand(
            driveBack,
            swerve::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(0.5, 0, 0),
            new PIDController(0.5, 0, 0),
            new PIDController(0.5, 0, 0),
            swerve::setModuleStates,
            swerve);

    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(driveBack.getInitialHolonomicPose())),
        driveBackCommand,
        new InstantCommand(
            () -> swerve.setYaw(driveBack.getEndState().holonomicRotation.getDegrees())));
  }
}
