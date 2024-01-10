package frc3512.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc3512.robot.Constants;
import frc3512.robot.subsystems.Swerve;
import frc3512.robot.subsystems.Vision;
import frc3512.lib.logging.SpartanDoubleEntry;

public class AprilTagAlign extends CommandBase {

  private final SpartanDoubleEntry targetVelocityEntry =
      new SpartanDoubleEntry("/Diagnostics/Vision/targetVelocity");
  private final SpartanDoubleEntry targetYawEntry =
      new SpartanDoubleEntry("/Diagnostics/Vision/targetYawEntry");
  private final Swerve swerve;
  private final Vision vision;
  private final PIDController thetaController =
      new PIDController(Constants.VisionConstants.turnP, 0.0, 0.0);

  public AprilTagAlign(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;
    // creating an object to take in our swerve and position

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    // how close we can be to our goal

    addRequirements(swerve);
    // adds a requirement to the command to have this subsystem
    // this also tells the scheduler to only run one thing at a time
  }

  @Override
  public void initialize() {
    //thetaController.reset(swerve.getPose().getRotation().getRadians());
    // reset the PID controller to the rotation of the robot
  }

  @Override
  public void execute() {
    if (vision.photonCamera.getLatestResult().hasTargets()){
      double targetYaw = vision.getBestTarget().getYaw();
      double thetaVelocity = -thetaController.calculate(targetYaw, 0);
      targetYawEntry.set(targetYaw);
      targetVelocityEntry.set(thetaVelocity);
      // determines how far we should move to get to our position
  
      //if (atGoal()) {
      //  thetaVelocity = 0.0;
      //}
  
      swerve.drive(new Translation2d(), thetaVelocity, false, false);
      // this is the code that actually drives the robot to the correct position based on the velocities
    }
  }

  //public boolean atGoal() {
  //  return (thetaController.atGoal());
  //}
}