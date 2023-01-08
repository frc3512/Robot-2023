package frc3512.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.motion.SpartanCANCoder;
import frc3512.lib.motion.SpartanSparkMax;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.lib.util.SwerveModuleConstants;
import frc3512.robot.Constants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private SpartanSparkMax angleMotor;
  private SpartanSparkMax driveMotor;
  private SpartanCANCoder angleEncoder;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SwerveConstants.driveKS,
          Constants.SwerveConstants.driveKV,
          Constants.SwerveConstants.driveKA);

  private final SpartanDoubleEntry cancoderReading;
  private final SpartanDoubleEntry integratedReading;
  private final SpartanDoubleEntry velocityReading;

  /**
   * Creates a new swerve module with NEO motors and a CTRE CANCoder.
   *
   * @param moduleNumber - Number of the module (0-3)
   * @param moduleIO - Module IO for the appropriate module
   */
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    angleEncoder =
        new SpartanCANCoder(moduleConstants.cancoderID, Constants.SwerveConstants.canCoderInvert);

    angleMotor =
        new SpartanSparkMax(
            moduleConstants.angleMotorID,
            Constants.SwerveConstants.angleInvert,
            Usage.kPositionOnly);
    configAngleMotor();

    driveMotor =
        new SpartanSparkMax(
            moduleConstants.driveMotorID, Constants.SwerveConstants.driveInvert, Usage.kAll);
    configDriveMotor();

    cancoderReading =
        new SpartanDoubleEntry("/Diagnostics/Swerve/Mod " + moduleNumber + "/CANCoder", 0.0, true);
    integratedReading =
        new SpartanDoubleEntry(
            "/Diagnostics/Swerve/Mod " + moduleNumber + "/Integrated", 0.0, true);
    velocityReading =
        new SpartanDoubleEntry("/Diagnostics/Swerve/Mod " + moduleNumber + "/Velocity", 0.0, true);

    lastAngle = getState().angle;
  }

  private void configAngleMotor() {
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    angleMotor.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    angleMotor.setPID(
        Constants.SwerveConstants.angleKP,
        Constants.SwerveConstants.angleKI,
        Constants.SwerveConstants.angleKD,
        Constants.SwerveConstants.angleKFF);
    angleMotor.enableContinuousInput(0.0, 360.0);
    angleMotor.enableVoltageComp(Constants.GeneralConstants.voltageCompSwerve);
    angleMotor.burnFlash();
    resetAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveMotor.setConversionFactors(
        Constants.SwerveConstants.driveConversionPositionFactor,
        Constants.SwerveConstants.driveConversionVelocityFactor);
    driveMotor.setPID(
        Constants.SwerveConstants.driveKP,
        Constants.SwerveConstants.driveKI,
        Constants.SwerveConstants.driveKD,
        Constants.SwerveConstants.driveKFF);
    driveMotor.enableVoltageComp(Constants.GeneralConstants.voltageCompSwerve);
    driveMotor.burnFlash();
    driveMotor.setPosition(0.0);
  }

  private void resetAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    angleMotor.setPosition(absolutePosition);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveMotor.performVelocityControl(
          desiredState.speedMetersPerSecond,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleMotor.performPositionControl(angle.getDegrees());
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angleMotor.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public void stop() {
    driveMotor.stop();
    angleMotor.stop();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(driveMotor.getPosition(), getAngle());
  }

  public void periodic() {
    cancoderReading.set(getCanCoder().getDegrees());
    integratedReading.set(getAngle().getDegrees());
    velocityReading.set(driveMotor.getVelocity());
  }
}
