package frc3512.lib.swervelib.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc3512.lib.swervelib.parser.PIDFConfig;

public class SparkMaxSwerve extends SwerveMotor {

  /** SparkMAX Instance. */
  public CANSparkMax motor;
  /** Integrated encoder. */
  public RelativeEncoder encoder;
  /** Closed-loop PID controller. */
  public SparkMaxPIDController pid;
  /** Factory default already occurred. */
  private boolean factoryDefaultOccurred = false;

  /**
   * Initialize the swerve motor.
   *
   * @param id CAN ID of the SparkMax
   * @param isDriveMotor Is the motor being initialized a drive motor?
   */
  public SparkMaxSwerve(int id, boolean isDriveMotor) {
    this.isDriveMotor = isDriveMotor;
    motor = new CANSparkMax(id, MotorType.kBrushless);
    factoryDefaults();
    clearStickyFaults();

    encoder = motor.getEncoder();
    pid = motor.getPIDController();
    pid.setFeedbackDevice(
        encoder); // Configure feedback of the PID controller as the integrated encoder.

    // Taken from
    // https://github.com/frc3512/SwerveBot-2022/blob/9d31afd05df6c630d5acb4ec2cf5d734c9093bf8/src/main/java/frc/lib/util/CANSparkMaxUtil.java#L67
    configureCANStatusFrames(10, 20, 20, 500, 500);

    motor.setCANTimeout(0); // Spin off configurations in a different thread.
  }

  /**
   * Set the voltage compensation for the swerve module motor.
   *
   * @param nominalVoltage Nominal voltage for operation to output to.
   */
  @Override
  public void setVoltageCompensation(double nominalVoltage) {
    motor.enableVoltageCompensation(nominalVoltage);
  }

  /**
   * Set the current limit for the swerve drive motor, remember this may cause jumping if used in
   * conjunction with voltage compensation. This is useful to protect the motor from current spikes.
   *
   * @param currentLimit Current limit in AMPS at free speed.
   */
  @Override
  public void setCurrentLimit(int currentLimit) {
    motor.setSmartCurrentLimit(currentLimit);
  }

  /**
   * Set the maximum rate the open/closed loop output can change by.
   *
   * @param rampRate Time in seconds to go from 0 to full throttle.
   */
  @Override
  public void setLoopRampRate(double rampRate) {
    motor.setOpenLoopRampRate(rampRate);
    motor.setClosedLoopRampRate(rampRate);
  }

  /** Configure the factory defaults. */
  @Override
  public void factoryDefaults() {
    if (!factoryDefaultOccurred) {
      motor.restoreFactoryDefaults();
      factoryDefaultOccurred = true;
    }
  }

  /** Clear the sticky faults on the motor controller. */
  @Override
  public void clearStickyFaults() {
    motor.clearFaults();
  }

  /**
   * Configure the integrated encoder for the swerve module. Sets the conversion factors for
   * position and velocity.
   *
   * @param positionConversionFactor The conversion factor to apply.
   */
  @Override
  public void configureIntegratedEncoder(double positionConversionFactor) {
    encoder.setPositionConversionFactor(positionConversionFactor);
    encoder.setVelocityConversionFactor(positionConversionFactor / 60);
  }

  /**
   * Configure the PIDF values for the closed loop controller.
   *
   * @param config Configuration class holding the PIDF values.
   */
  @Override
  public void configurePIDF(PIDFConfig config) {
    int pidSlot =
        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    pid.setP(config.p, pidSlot);
    pid.setI(config.i, pidSlot);
    pid.setD(config.d, pidSlot);
    pid.setFF(config.f, pidSlot);
    pid.setIZone(config.iz, pidSlot);
  }

  /**
   * Configure the PID wrapping for the position closed loop controller.
   *
   * @param minInput Minimum PID input.
   * @param maxInput Maximum PID input.
   */
  @Override
  public void configurePIDWrapping(double minInput, double maxInput) {
    pid.setPositionPIDWrappingEnabled(true);
    pid.setPositionPIDWrappingMinInput(minInput);
    pid.setPositionPIDWrappingMaxInput(maxInput);
  }

  /**
   * Set the CAN status frames.
   *
   * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
   * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
   * @param CANStatus2 Motor Position
   * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
   * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
   */
  public void configureCANStatusFrames(
      int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4) {
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, CANStatus0);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, CANStatus1);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, CANStatus2);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, CANStatus3);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, CANStatus4);
    // May need to configure Status Frame 5 and 6 if necessary
    //  https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  }

  /**
   * Set the idle mode.
   *
   * @param isBrakeMode Set the brake mode.
   */
  @Override
  public void setMotorBrake(boolean isBrakeMode) {
    motor.setIdleMode(isBrakeMode ? IdleMode.kBrake : IdleMode.kCoast);
  }

  /**
   * Set the motor to be inverted.
   *
   * @param inverted State of inversion.
   */
  @Override
  public void setInverted(boolean inverted) {
    motor.setInverted(inverted);
  }

  /** Save the configurations from flash to EEPROM. */
  @Override
  public void burnFlash() {
    motor.burnFlash();
  }

  /**
   * Set the percentage output.
   *
   * @param percentOutput percent out for the motor controller.
   */
  @Override
  public void set(double percentOutput) {
    motor.set(percentOutput);
  }

  /**
   * Set the closed loop PID controller reference point.
   *
   * @param setpoint Setpoint in MPS or Angle in degrees.
   * @param feedforward Feedforward in volt-meter-per-second or kV.
   */
  @Override
  public void setReference(double setpoint, double feedforward) {
    int pidSlot =
        isDriveMotor ? SparkMAX_slotIdx.Velocity.ordinal() : SparkMAX_slotIdx.Position.ordinal();
    pid.setReference(
        setpoint,
        isDriveMotor ? ControlType.kVelocity : ControlType.kPosition,
        pidSlot,
        feedforward);
  }

  /**
   * Get the velocity of the integrated encoder.
   *
   * @return velocity
   */
  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Get the position of the integrated encoder.
   *
   * @return Position
   */
  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Set the integrated encoder position.
   *
   * @param position Integrated encoder position.
   */
  @Override
  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  /** REV Slots for PID configuration. */
  enum SparkMAX_slotIdx {
    Position,
    Velocity,
    Simulation
  }
}
