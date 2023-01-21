package frc3512.lib.motion;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import frc3512.lib.util.CANCoderUtil;
import frc3512.lib.util.CANCoderUtil.CANCoderUsage;

/* Wrapper class around the CANCoder. */
public class SpartanCANCoder {

  private CANCoder cancoder;
  private CANCoderConfiguration config;

  boolean inverted;
  CANCoderUsage usage;

  /**
   * Creates a new SpartanCANCoder.
   *
   * @param id CAN ID for the device
   * @param invert Whether to invert the encoder or not
   */
  public SpartanCANCoder(int id, boolean invert, CANCoderUsage usage) {
    this.inverted = invert;
    this.usage = usage;
    cancoder = new CANCoder(id);
    config = new CANCoderConfiguration();
    buildConfig();
    configEncoder();
  }

  /**
   * Creates a new SpartanCANCoder;
   *
   * @param id CAN ID for the device
   * @param canivore Name of the Canivore
   * @param invert Whether to invert the encoder or not
   */
  public SpartanCANCoder(int id, String canivore, boolean invert, CANCoderUsage usage) {
    this.inverted = invert;
    this.usage = usage;
    cancoder = new CANCoder(id, canivore);
    config = new CANCoderConfiguration();
    buildConfig();
    configEncoder();
  }

  private void configEncoder() {
    cancoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(cancoder, usage);
    cancoder.configAllSettings(config);
  }

  private void buildConfig() {
    config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
    config.sensorDirection = inverted;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
  }

  public double getPosition() {
    return cancoder.getPosition();
  }

  public double getVelocity() {
    return cancoder.getVelocity();
  }

  public double getAbsolutePosition() {
    return cancoder.getAbsolutePosition();
  }
}
