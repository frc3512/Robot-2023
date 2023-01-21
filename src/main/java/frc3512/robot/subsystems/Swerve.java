package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.logging.SpartanPose2dEntry;
import frc3512.lib.sim.GyroSim;
import frc3512.robot.Constants;

public class Swerve extends SubsystemBase {
  private final WPI_Pigeon2 gyro;
  private final GyroSim gyroSim;
  private final Vision vision;

  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveDriveOdometry odometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;
  private final SpartanDoubleEntry gyroYaw;
  private final SpartanPose2dEntry odometryPose;

  /** Subsystem class for the swerve drive. */
  public Swerve(Vision vision) {
    this.vision = vision;

    gyro = new WPI_Pigeon2(Constants.SwerveConstants.pigeonID);
    gyroSim = new GyroSim(gyro);
    gyro.configFactoryDefault();
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

    odometry =
        new SwerveDriveOdometry(
            Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(), new Pose2d());

    field = new Field2d();
    SmartDashboard.putData("Field", field);
    gyroYaw = new SpartanDoubleEntry("/Diagnostics/Swerve/Gyro/Yaw", 0.0);
    odometryPose = new SpartanPose2dEntry("/Diagnostics/Swerve/Odometry", new Pose2d());
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

  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    odometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void zeroGyro() {
    setYaw(0.0);
  }

  public void setYaw(double degrees) {
    gyro.setYaw(degrees);
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
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

  public Rotation2d getYaw() {
    return (Constants.SwerveConstants.invertGyro)
        ? Rotation2d.fromDegrees(360 - gyro.getYaw())
        : Rotation2d.fromDegrees(gyro.getYaw());
  }

  @Override
  public void periodic() {
    poseEstimator.update(getYaw(), getPositions());
    odometry.update(getYaw(), getPositions());

    /* BROKEN, BUG FIX NEEDED
    if (RobotBase.isReal()) {
      var result = vision.getEstimatedGlobalPose(getEstimatedPose());
      if (result.isPresent()) {
        poseEstimator.addVisionMeasurement(
            result.get().estimatedPose.toPose2d(),
            Timer.getFPGATimestamp() - result.get().timestampSeconds);
      }
    }
    */

    vision.setRobotPose(getEstimatedPose());
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
    gyroSim.setYaw(chassisSpeeds.omegaRadiansPerSecond);
  }
}
