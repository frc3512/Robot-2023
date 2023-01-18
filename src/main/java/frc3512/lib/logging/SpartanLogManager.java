package frc3512.lib.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import frc3512.robot.Constants;

/** Wrapper class around the DataLogManager for additional features. */
public class SpartanLogManager {

  private static DataLog log = DataLogManager.getLog();
  private static NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();

  /** Start logging (if running on real hardware). */
  public static void startLogging() {
    DataLogManager.logNetworkTables(false);
    DataLogManager.start();
  }

  /**
   * Returns the DataLog object for creating custom log entries.
   *
   * @return Instance of the DataLog.
   */
  public static DataLog getCurrentLog() {
    return log;
  }

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
