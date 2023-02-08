package frc3512.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.swervelib.SwerveDrive;
import frc3512.lib.swervelib.SwerveModule.Verbosity;
import frc3512.lib.swervelib.SwerveParser;
import frc3512.robot.Constants;
import java.io.File;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.photonvision.EstimatedRobotPose;

public class Swerve extends SubsystemBase {

  private final Timer syncTimer = new Timer();
  private final Vision vision;
  private final SwerveDrive swerve;

  /** Subsystem class for the swerve drive. */
  public Swerve(Vision vision) {
    this.vision = vision;

    swerve = SwerveParser.fromJSONDirectory(new File(Filesystem.getDeployDirectory(), "swerve"));
    swerve.zeroGyro();
    swerve.setDeadband(Constants.GeneralConstants.swerveDeadband);
    swerve.setSafetyEnabled(false);

    SmartDashboard.putData(swerve);
  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
          swerve.drive(
              translationSup.getAsDouble(),
              strafeSup.getAsDouble(),
              rotationSup.getAsDouble(),
              true);
        })
        .withName("TeleopSwerve");
  }

  public Command followTrajectory(PathPlannerTrajectory trajectory, boolean firstPath) {
    return Commands.sequence(
        run(
            () -> {
              if (firstPath) {
                swerve.resetOdometry(trajectory.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            trajectory,
            swerve::getPose,
            new PIDController(Constants.AutonConstants.xControllerP, 0, 0),
            new PIDController(Constants.AutonConstants.yControllerP, 0, 0),
            new PIDController(Constants.AutonConstants.thetaControllerP, 0, 0),
            swerve::set,
            this),
        this.drive(() -> 0.0, () -> 0.0, () -> 0.0));
  }

  public void zeroGyro() {
    swerve.zeroGyro();
  }

  @Override
  public void periodic() {
    if (syncTimer.advanceIfElapsed(1.0)) {
      swerve.publish(Verbosity.HIGH);
    }
    if (SmartDashboard.getBoolean("Update Swerve Drive", false)) {
      SmartDashboard.putBoolean("Update Swerve Drive", false);
      swerve.subscribe();
    }

    swerve.update();

    if (RobotBase.isReal()) {
      Optional<EstimatedRobotPose> result = vision.getEstimatedGlobalPose(swerve.getPose());

      if (result.isPresent()) {
        EstimatedRobotPose camPose = result.get();
        swerve.addVisionMeasurement(camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      }
    }
  }
}
