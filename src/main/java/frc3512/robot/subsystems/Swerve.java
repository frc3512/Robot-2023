package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.logging.SpartanPose2dEntry;
import frc3512.lib.sim.GyroSim;
import frc3512.robot.Constants;
import java.util.function.DoubleSupplier;

public class Swerve extends SubsystemBase {
  private final WPI_Pigeon2 gyro;
  private final GyroSim gyroSim;
  private final Vision vision;

  private SwerveDrivePoseEstimator swervePoseEstimator;
  private SwerveModule[] mSwerveMods;

  private Field2d field;
  private final SpartanDoubleEntry gyroYaw;
  private final SpartanPose2dEntry odometryPose;

  public PIDController controller = new PIDController(0.1, 0.0, 0.0);

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  /** Subsystem class for the swerve drive. */
  public Swerve(Vision vision) {
    this.vision = vision;

    gyro = new WPI_Pigeon2(Constants.SwerveConstants.pigeonID);
    gyroSim = new GyroSim(gyro);
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

  public CommandBase driveWithJoysticks(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {

    double translationVal =
        translationLimiter.calculate(
            MathUtil.applyDeadband(
                translationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
    double strafeVal =
        strafeLimiter.calculate(
            MathUtil.applyDeadband(
                strafeSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));
    double rotationVal =
        rotationLimiter.calculate(
            MathUtil.applyDeadband(
                rotationSup.getAsDouble(), Constants.GeneralConstants.swerveDeadband));

    return run(() ->
            drive(
                new Translation2d(translationVal, strafeVal)
                    .times(Constants.SwerveConstants.maxSpeed),
                rotationVal * Constants.SwerveConstants.maxAngularVelocity,
                true,
                true))
        .withName("DriveSwerveWithJoysticks");
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void resetOdometry(Pose2d pose) {
    swervePoseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Pose2d getPose() {
    return swervePoseEstimator.getEstimatedPosition();
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
    swervePoseEstimator.update(getYaw(), getPositions());

    if (RobotBase.isReal()) {
      swervePoseEstimator.addVisionMeasurement(
          vision.estimateGlobalPose(getPose()), vision.getGlobalTimestamp());
    }

    vision.setRobotPose(getPose());
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
