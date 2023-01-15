package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.logging.SpartanPose2dEntry;
import frc3512.robot.Constants;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;
  private double yawSim = 0.0;
  private BasePigeonSimCollection gyroSim;

  private final Vision vision;

  private SwerveDrivePoseEstimator swervePoseEstimator;
  private SwerveModule[] mSwerveMods;

  private Field2d field;
  private final SpartanDoubleEntry gyroYaw;
  private final SpartanPose2dEntry odometryPose;

  public PIDController controller = new PIDController(0.1, 0.0, 0.0);

  /** Subsystem class for the swerve drive. */
  public Swerve(Vision vision) {
    this.vision = vision;
    gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
    gyroSim = gyro.getSimCollection();
    gyro.configFactoryDefault();
    zeroGyro();
    controller.setTolerance(4.5);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

    swervePoseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(), new Pose2d());
    field = new Field2d();
    SmartDashboard.putData("Field", field);
    gyroYaw = new SpartanDoubleEntry("/Diagnostics/Swerve/Gyro/Yaw", 0.0, true);
    odometryPose = new SpartanPose2dEntry("/Diagnostics/Swerve/Odometry", new Pose2d(), true);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public double getDistanceFromTarget(Pose2d targetPose) {
    return vision.getRange(getPose(), targetPose);
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public void stop() {
    for (SwerveModule mod : mSwerveMods) {
      mod.stop();
    }
  }

  public Rotation2d getYaw() {
    return (Constants.SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    swervePoseEstimator.update(getYaw(), getPositions());

    if (RobotBase.isReal() && vision.hasTargets()) {
      var camPose = vision.estimateGlobalPose(getPose());

      if (camPose.get() != null) {
        swervePoseEstimator.addVisionMeasurement(
            camPose.get().estimatedPose.toPose2d(), camPose.get().timestampSeconds);
      }
    }

    field.setRobotPose(getPose());

    mSwerveMods[0].periodic();
    mSwerveMods[1].periodic();
    mSwerveMods[2].periodic();
    mSwerveMods[3].periodic();
    gyroYaw.set(getYaw().getDegrees());
    odometryPose.set(getPose());
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();

    ChassisSpeeds chassisSpeeds =
        Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
    yawSim += chassisSpeeds.omegaRadiansPerSecond * 0.02;

    gyroSim.setRawHeading(-Units.radiansToDegrees(yawSim) - getYaw().getDegrees());
  }
}
