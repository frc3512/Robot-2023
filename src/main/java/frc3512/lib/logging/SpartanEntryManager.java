package frc3512.lib.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc3512.robot.Constants;

/** Wrapper class around the DataLogManager for additional features. */
public class SpartanEntryManager {

  private static NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

  /**
   * Returns the NetworkTable default instance.
   *
   * @return Default NetworkTable instance.
   */
  public static NetworkTableInstance getNTInstance() {
    return ntInstance;
  }

  /**
   * Returns if the robot is set to run in tuning mode. Disables logging if thats the case.
   *
   * @return Whether or not tuning mode is enabled.
   */
  public static boolean isTuningMode() {
    return Constants.GeneralConstants.tuningMode;
  }
}
