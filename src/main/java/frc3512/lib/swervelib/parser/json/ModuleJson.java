package frc3512.lib.swervelib.parser.json;

import edu.wpi.first.math.util.Units;
import frc3512.lib.swervelib.parser.PIDFConfig;
import frc3512.lib.swervelib.parser.SwerveModuleConfiguration;
import frc3512.lib.swervelib.parser.SwerveModulePhysicalCharacteristics;
import frc3512.lib.swervelib.parser.json.modules.InvertedMotorJson;
import frc3512.lib.swervelib.parser.json.modules.LocationJson;

/**
 * {@link frc.robot.subsystems.swervedrive2.swervelib.SwerveModule} JSON parsed class. Used to
 * access the JSON data.
 */
public class ModuleJson {

  /** Drive motor device configuration. */
  public DeviceJson drive;
  /** Angle motor device configuration. */
  public DeviceJson angle;
  /** Absolute encoder device configuration. */
  public DeviceJson encoder;
  /** Defines which motors are inverted. */
  public InvertedMotorJson inverted;
  /** Absolute encoder offset from 0 in degrees. */
  public double absoluteEncoderOffset;
  /** The location of the swerve module from the center of the robot in inches. */
  public LocationJson location;

  /**
   * Create the swerve module configuration based off of parsed data.
   *
   * @param anglePIDF The PIDF values for the angle motor.
   * @param velocityPIDF The velocity PIDF values for the drive motor.
   * @param maxSpeed The maximum speed of the robot in meters per second.
   * @param physicalCharacteristics Physical characteristics of the swerve module.
   * @return {@link SwerveModuleConfiguration} based on the provided data and parsed data.
   */
  public SwerveModuleConfiguration createModuleConfiguration(
      PIDFConfig anglePIDF,
      PIDFConfig velocityPIDF,
      double maxSpeed,
      SwerveModulePhysicalCharacteristics physicalCharacteristics) {
    return new SwerveModuleConfiguration(
        drive.createMotor(true),
        angle.createMotor(false),
        encoder.createEncoder(),
        absoluteEncoderOffset,
        Units.inchesToMeters(location.x),
        Units.inchesToMeters(location.y),
        anglePIDF,
        velocityPIDF,
        maxSpeed,
        physicalCharacteristics);
  }
}
