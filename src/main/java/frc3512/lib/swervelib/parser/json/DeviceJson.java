package frc3512.lib.swervelib.parser.json;

import frc3512.lib.swervelib.encoders.CANCoderSwerve;
import frc3512.lib.swervelib.encoders.SwerveAbsoluteEncoder;
import frc3512.lib.swervelib.imu.Pigeon2Swerve;
import frc3512.lib.swervelib.imu.PigeonSwerve;
import frc3512.lib.swervelib.imu.SwerveIMU;
import frc3512.lib.swervelib.motors.SparkMaxSwerve;
import frc3512.lib.swervelib.motors.SwerveMotor;

/** Device JSON parsed class. Used to access the JSON data. */
public class DeviceJson {

  /** The device type, e.g. pigeon/pigeon2/sparkmax/talonfx/navx */
  public String type;
  /** The CAN ID or pin ID of the device. */
  public int id;
  /** The CAN bus name which the device resides on if using CAN. */
  public String canbus = "";

  /**
   * Create a {@link SwerveAbsoluteEncoder} from the current configuration.
   *
   * @return {@link SwerveAbsoluteEncoder} given.
   */
  public SwerveAbsoluteEncoder createEncoder() {
    switch (type) {
      case "integrated":
        return null;
      case "cancoder":
        return new CANCoderSwerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveIMU} from the given configuration.
   *
   * @return {@link SwerveIMU} given.
   */
  public SwerveIMU createIMU() {
    switch (type) {
      case "pigeon":
        return new PigeonSwerve(id);
      case "pigeon2":
        return new Pigeon2Swerve(id, canbus != null ? canbus : "");
      default:
        throw new RuntimeException(type + " is not a recognized absolute encoder type.");
    }
  }

  /**
   * Create a {@link SwerveMotor} from the given configuration.
   *
   * @param isDriveMotor If the motor being generated is a drive motor.
   * @return {@link SwerveMotor} given.
   */
  public SwerveMotor createMotor(boolean isDriveMotor) {
    if (type.equals("sparkmax")) {
      return new SparkMaxSwerve(id, isDriveMotor);
    }
    throw new RuntimeException(type + " is not a recognized absolute encoder type.");
  }
}
