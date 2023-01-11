package frc3512.lib.motion;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;

/* Wrapper class around the Spark Max motor controller.
 * Infers that you are using a NEO or NEO500 brushless motor.
 */
public class SpartanSparkMax {

  private CANSparkMax motor;
  private RelativeEncoder encoder;
  private SparkMaxPIDController controller;

  boolean inverted;
  Usage usage;

  /**
   * Creates a SpartanSparkMax.
   *
   * @param id CAN ID of the device
   * @param invert Whether to invert the motor or not.
   */
  public SpartanSparkMax(int id, boolean invert, Usage usage) {
    this.inverted = invert;
    this.usage = usage;
    motor = new CANSparkMax(id, MotorType.kBrushless);
    encoder = motor.getEncoder();
    controller = motor.getPIDController();

    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, usage);
    motor.setInverted(invert);
  }

  public void setPosition(double position) {
    encoder.setPosition(position);
  }

  public void setVelocityConverstionFactor(double factor) {
    encoder.setVelocityConversionFactor(factor);
  }

  public void setPositionConversionFactor(double factor) {
    encoder.setPositionConversionFactor(factor);
  }

  public void setConversionFactors(double positionFactor, double velocityFactor) {
    encoder.setPositionConversionFactor(positionFactor);
    encoder.setVelocityConversionFactor(velocityFactor);
  }

  public void setIdleMode(IdleMode mode) {
    motor.setIdleMode(mode);
  }

  public void setSmartCurrentLimit(int limit) {
    motor.setSmartCurrentLimit(limit);
  }

  public void enableVoltageComp(double value) {
    motor.enableVoltageCompensation(value);
  }

  public void enableContinuousInput(double min, double max) {
    controller.setPositionPIDWrappingEnabled(true);
    controller.setPositionPIDWrappingMinInput(min);
    controller.setPositionPIDWrappingMaxInput(max);
  }

  public void setPID(double p, double i, double d, double ff) {
    controller.setP(p);
    controller.setI(i);
    controller.setD(d);
    controller.setFF(ff);
  }

  public void setPID(double p, double i, double d) {
    setPID(p, i, d, 0.0);
  }

  public void burnFlash() {
    motor.burnFlash();
  }

  public void set(double percent) {
    motor.set(percent);
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void performVelocityControl(double velocity, double feedforward) {
    controller.setReference(velocity, ControlType.kVelocity, 0, feedforward);
  }

  public void performVelocityControl(double velocity) {
    controller.setReference(velocity, ControlType.kVelocity);
  }

  public void performPositionControl(double angle) {
    controller.setReference(angle, ControlType.kPosition);
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public double getPosition() {
    return encoder.getPosition();
  }
}
