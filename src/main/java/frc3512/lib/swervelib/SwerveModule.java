package frc3512.lib.swervelib;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.swervelib.encoders.SwerveAbsoluteEncoder;
import frc3512.lib.swervelib.math.SwerveModuleState2;
import frc3512.lib.swervelib.motors.SwerveMotor;
import frc3512.lib.swervelib.parser.SwerveModuleConfiguration;
import frc3512.robot.Robot;

public class SwerveModule {

  /** Swerve module configuration options. */
  public final SwerveModuleConfiguration configuration;
  /** Angle offset from the absolute encoder. */
  private final double angleOffset;
  /** Swerve Motors. */
  private final SwerveMotor angleMotor, driveMotor;
  /** Absolute encoder for swerve drive. */
  private final SwerveAbsoluteEncoder absoluteEncoder;
  /**
   * Module number for kinematics, usually 0 to 3. front left -> front right -> back left -> back
   * right.
   */
  public int moduleNumber;
  /** Feedforward for drive motor during closed loop control. */
  public SimpleMotorFeedforward feedforward;
  /** Last angle set for the swerve module. */
  private double lastAngle;
  /** Current state. */
  private double angle, omega, speed, fakePos, lastTime, dt;
  /** Timer for simulation. */
  private Timer time;
  /** NetworkTable entries */
  private SpartanDoubleEntry speedEntry,
      angleEntry,
      omegaEntry,
      azimuthEntry,
      relativeEncoderEntry,
      absoluteEncoderEntry;

  /**
   * Construct the swerve module and initialize the swerve module motors and absolute encoder.
   *
   * @param moduleNumber Module number for kinematics.
   * @param moduleConfiguration Module constants containing CAN ID's and offsets.
   */
  public SwerveModule(int moduleNumber, SwerveModuleConfiguration moduleConfiguration) {
    angle = 0;
    speed = 0;
    omega = 0;
    fakePos = 0;
    this.moduleNumber = moduleNumber;
    configuration = moduleConfiguration;
    angleOffset = moduleConfiguration.angleOffset;

    // Initialize Feedforward for drive motor.
    feedforward = configuration.createDriveFeedforward();

    // Create motors from configuration and reset them to defaults.
    angleMotor = moduleConfiguration.angleMotor;
    driveMotor = moduleConfiguration.driveMotor;
    angleMotor.factoryDefaults();
    driveMotor.factoryDefaults();

    // Configure voltage comp, current limit, and ramp rate.
    angleMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    driveMotor.setVoltageCompensation(configuration.physicalCharacteristics.optimalVoltage);
    angleMotor.setCurrentLimit(configuration.physicalCharacteristics.angleMotorCurrentLimit);
    driveMotor.setCurrentLimit(configuration.physicalCharacteristics.driveMotorCurrentLimit);
    angleMotor.setLoopRampRate(configuration.physicalCharacteristics.angleMotorRampRate);
    driveMotor.setLoopRampRate(configuration.physicalCharacteristics.driveMotorRampRate);

    // Config angle encoders
    absoluteEncoder = moduleConfiguration.absoluteEncoder;
    absoluteEncoder.factoryDefault();
    absoluteEncoder.configure(moduleConfiguration.absoluteEncoderInverted);
    angleMotor.configureIntegratedEncoder(moduleConfiguration.getPositionEncoderConversion(false));
    angleMotor.setPosition(absoluteEncoder.getAbsolutePosition() - angleOffset);

    // Config angle motor/controller
    angleMotor.configurePIDF(moduleConfiguration.anglePIDF);
    angleMotor.configurePIDWrapping(-180, 180);
    angleMotor.setMotorBrake(false);

    // Config drive motor/controller
    driveMotor.configureIntegratedEncoder(moduleConfiguration.getPositionEncoderConversion(true));
    driveMotor.configurePIDF(moduleConfiguration.velocityPIDF);
    driveMotor.setInverted(moduleConfiguration.driveMotorInverted);
    driveMotor.setMotorBrake(true);

    driveMotor.burnFlash();
    angleMotor.burnFlash();

    lastAngle = getState().angle.getDegrees();

    if (!Robot.isReal()) {
      time = new Timer();
      time.start();
      lastTime = time.get();
    }

    speedEntry =
        new SpartanDoubleEntry(
            "/Diagnostics/Swerve/Optimized Module #" + moduleNumber + "/Speed Setpoint");
    angleEntry =
        new SpartanDoubleEntry(
            "/Diagnostics/Swerve/Optimized Module #" + moduleNumber + "/Angle Setpoint");
    omegaEntry =
        new SpartanDoubleEntry(
            "/Diagnostics/Swerve/Optimized Module #" + moduleNumber + "/Omega Setpoint");
    azimuthEntry =
        new SpartanDoubleEntry("/Diagnostics/Swerve/Module #" + moduleNumber + "/Azimuth Angle");

    relativeEncoderEntry =
        new SpartanDoubleEntry(
            "/Diagnostics/Swerve/Module #" + moduleNumber + "/Relative Encoder Reading");
    absoluteEncoderEntry =
        new SpartanDoubleEntry(
            "/Diagnostics/Swerve/Module #" + moduleNumber + "/Absolute Encoder Reading");
  }

  /** Synchronize the integrated angle encoder with the absolute encoder. */
  public void synchronizeEncoders() {
    angleMotor.setPosition(absoluteEncoder.getAbsolutePosition() - angleOffset);
  }

  /**
   * Set the desired state of the swerve module.
   *
   * @param desiredState Desired swerve module state.
   * @param isOpenLoop Whether to use open loop (direct percent) or direct velocity control.
   */
  public void setDesiredState(SwerveModuleState2 desiredState, boolean isOpenLoop) {
    SwerveModuleState simpleState =
        new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    simpleState = SwerveModuleState.optimize(simpleState, getState().angle);
    desiredState =
        new SwerveModuleState2(
            simpleState.speedMetersPerSecond, simpleState.angle, desiredState.omegaRadPerSecond);

    speedEntry.set(desiredState.speedMetersPerSecond);
    angleEntry.set(desiredState.angle.getDegrees());
    omegaEntry.set(desiredState.omegaRadPerSecond);

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / configuration.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      double velocity = desiredState.speedMetersPerSecond;
      driveMotor.setReference(velocity, feedforward.calculate(velocity));
    }

    // Prevents module rotation if speed is less than 1%
    double angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (configuration.maxSpeed * 0.01)
            ? lastAngle
            : desiredState.angle.getDegrees());
    angleMotor.setReference(
        angle, Math.toDegrees(desiredState.omegaRadPerSecond) * configuration.angleKV);
    lastAngle = angle;

    if (!Robot.isReal()) {
      dt = time.get() - lastTime;
      fakePos += (speed * dt);
      lastTime = time.get();
    }

    this.angle = desiredState.angle.getDegrees();
    omega = desiredState.omegaRadPerSecond;
    speed = desiredState.speedMetersPerSecond;

    relativeEncoderEntry.set(getRelativeEncoder());
    absoluteEncoderEntry.set(getCANCoder());
  }

  /**
   * Get the Swerve Module state.
   *
   * @return Current SwerveModule state.
   */
  public SwerveModuleState2 getState() {
    double velocity;
    Rotation2d azimuth;
    double omega;
    if (Robot.isReal()) {
      velocity = driveMotor.getVelocity();
      azimuth = Rotation2d.fromDegrees(angleMotor.getPosition());
      omega = angleMotor.getVelocity();
    } else {
      velocity = speed;
      azimuth = Rotation2d.fromDegrees(this.angle);
      omega = this.omega;
    }
    return new SwerveModuleState2(velocity, azimuth, omega);
  }

  public SwerveModulePosition getPosition() {
    double position;
    Rotation2d azimuth;
    if (Robot.isReal()) {
      position = driveMotor.getPosition();
      azimuth = Rotation2d.fromDegrees(angleMotor.getPosition());
    } else {
      position = fakePos;
      azimuth = Rotation2d.fromDegrees(angle + (Math.toDegrees(omega) * dt));
    }
    azimuthEntry.set(azimuth.getDegrees());
    return new SwerveModulePosition(position, azimuth);
  }

  /**
   * Get the CANCoder absolute position.
   *
   * @return Absolute encoder angle in degrees.
   */
  public double getCANCoder() {
    return absoluteEncoder.getAbsolutePosition();
  }

  /**
   * Get the relative encoder angle in degrees.
   *
   * @return Angle in degrees.
   */
  public double getRelativeEncoder() {
    return angleMotor.getPosition();
  }

  /**
   * Set the brake mode.
   *
   * @param brake Set the brake mode.
   */
  public void setMotorBrake(boolean brake) {
    driveMotor.setMotorBrake(brake);
  }
}
