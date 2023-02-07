package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleArrayEntry;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.sim.GyroSim;
import frc3512.lib.util.PhotonCameraWrapper;
import frc3512.robot.Constants;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;

public class Swerve extends SubsystemBase {
  private final WPI_Pigeon2 gyro;
  private final GyroSim gyroSim;
  private final PhotonCameraWrapper camera;

  private SwerveDrivePoseEstimator poseEstimator;
  private SwerveModule[] mSwerveMods;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private Field2d field;
  private final SpartanDoubleEntry gyroYaw;
  private final SpartanDoubleArrayEntry moduleIntegratedPositions;
  private final SpartanDoubleArrayEntry moduleAbsolutePositions;
  private final SpartanDoubleArrayEntry moduleDriveVelocities;
  private final SpartanDoubleArrayEntry moduleDrivePositions;

  /** Subsystem class for the swerve drive. */
  public Swerve() {
    camera = new PhotonCameraWrapper();
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

    poseEstimator =
        new SwerveDrivePoseEstimator(
            Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(), new Pose2d());

    field = new Field2d();
    SmartDashboard.putData("Field", field);

    gyroYaw = new SpartanDoubleEntry("/Diagnostics/Swerve/Gyro/Yaw");
    moduleIntegratedPositions =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Integrated Positions");
    moduleAbsolutePositions =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Absolute Positions");
    moduleDriveVelocities =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Drive Velocity");
    moduleDrivePositions =
        new SpartanDoubleArrayEntry("/Diagnostics/Swerve/Modules/Drive Positions");
  }

  public Command drive(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    return run(() -> {
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

          drive(
              new Translation2d(translationVal, strafeVal)
                  .times(Constants.SwerveConstants.maxSpeed),
              rotationVal * Constants.SwerveConstants.maxAngularVelocity,
              true,
              true);
        })
        .withName("TeleopSwerve");
  }

  public Command followTrajectory(PathPlannerTrajectory trajectory, boolean firstPath) {
    return Commands.sequence(
        run(
            () -> {
              if (firstPath) {
                this.resetOdometry(trajectory.getInitialHolonomicPose());
              }
            }),
        new PPSwerveControllerCommand(
            trajectory,
            this::getPose,
            Constants.SwerveConstants.swerveKinematics,
            new PIDController(Constants.AutonConstants.xControllerP, 0, 0),
            new PIDController(Constants.AutonConstants.yControllerP, 0, 0),
            new PIDController(Constants.AutonConstants.thetaControllerP, 0, 0),
            this::setModuleStates,
            this),
        this.drive(() -> 0.0, () -> 0.0, () -> 0.0));
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
  }

  public void zeroGyro() {
    gyro.reset();
  }

  public void setYaw(double degrees) {
    gyro.setYaw(degrees);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
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

    Optional<EstimatedRobotPose> result = camera.getEstimatedGlobalPose(getPose());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(), Timer.getFPGATimestamp() - camPose.timestampSeconds);
    }

    moduleAbsolutePositions.set(
        new double[] {
          mSwerveMods[0].getCanCoder().getDegrees(),
          mSwerveMods[1].getCanCoder().getDegrees(),
          mSwerveMods[2].getCanCoder().getDegrees(),
          mSwerveMods[3].getCanCoder().getDegrees()
        });
    moduleIntegratedPositions.set(
        new double[] {
          mSwerveMods[0].getAnglePosition().getDegrees(),
          mSwerveMods[1].getAnglePosition().getDegrees(),
          mSwerveMods[2].getAnglePosition().getDegrees(),
          mSwerveMods[3].getAnglePosition().getDegrees()
        });
    moduleDriveVelocities.set(
        new double[] {
          mSwerveMods[0].getDriveVelocity(),
          mSwerveMods[1].getDriveVelocity(),
          mSwerveMods[2].getDriveVelocity(),
          mSwerveMods[3].getDriveVelocity()
        });
    moduleDrivePositions.set(
        new double[] {
          mSwerveMods[0].getDrivePosition(),
          mSwerveMods[1].getDrivePosition(),
          mSwerveMods[2].getDrivePosition(),
          mSwerveMods[3].getDrivePosition()
        });

    gyroYaw.set(getYaw().getDegrees());
    field.setRobotPose(getPose());
  }

  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();

    ChassisSpeeds chassisSpeeds =
        Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
    gyroSim.setYaw(chassisSpeeds.omegaRadiansPerSecond);
  }
}
