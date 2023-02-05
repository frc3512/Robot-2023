package frc3512.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc3512.lib.sim.MotorSim;
import frc3512.lib.util.CANCoderUtil;
import frc3512.lib.util.CANCoderUtil.CANCoderUsage;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.lib.util.SwerveModuleConstants;
import frc3512.robot.Constants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SwerveConstants.driveKS,
          Constants.SwerveConstants.driveKV,
          Constants.SwerveConstants.driveKA);

  private final MotorSim driveMotorSim = new MotorSim();
  private final MotorSim angleMotorSim = new MotorSim();

  /**
   * Creates a new swerve module with NEO motors and a CTRE CANCoder.
   *
   * @param moduleNumber - Number of the module (0-3)
   * @param moduleIO - Module IO for the appropriate module
   */
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  private void configAngleEncoder() {
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = Constants.SwerveConstants.canCoderInvert;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;

    angleEncoder.configFactoryDefault();
    angleEncoder.configAllSettings(config);
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CANCoderUsage.kMinimal);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(
        Constants.SwerveConstants.angleConversionFactor);
    angleController.setP(Constants.SwerveConstants.angleKP);
    angleController.setI(Constants.SwerveConstants.angleKI);
    angleController.setD(Constants.SwerveConstants.angleKD);
    angleController.setFF(Constants.SwerveConstants.angleKFF);
    angleController.setPositionPIDWrappingMinInput(-180.0);
    angleController.setPositionPIDWrappingMaxInput(180.0);
    angleMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    angleMotor.burnFlash();
    resetAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setOpenLoopRampRate(Constants.SwerveConstants.openLoopRamp);
    driveMotor.setClosedLoopRampRate(Constants.SwerveConstants.closedLoopRamp);
    driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(
        Constants.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(
        Constants.SwerveConstants.driveConversionPositionFactor);
    driveController.setP(Constants.SwerveConstants.angleKP);
    driveController.setI(Constants.SwerveConstants.angleKI);
    driveController.setD(Constants.SwerveConstants.angleKD);
    driveController.setFF(Constants.SwerveConstants.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void resetAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    angleEncoder.setPosition(absolutePosition);
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
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;

    if (RobotBase.isSimulation()) {
      driveMotorSim.updateSimVelocity(desiredState);
      angleMotorSim.setCurrentAngle(angle.getDegrees());
    }
  }

  public Rotation2d getAnglePosition() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public void stop() {
    driveMotor.stopMotor();
    angleMotor.stopMotor();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAnglePosition());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getAnglePosition());
  }
}
