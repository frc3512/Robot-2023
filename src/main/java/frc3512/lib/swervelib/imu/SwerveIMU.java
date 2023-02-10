package frc3512.lib.swervelib.imu;

public abstract class SwerveIMU {

  /** Reset IMU to factory default. */
  public abstract void factoryDefault();

  /** Clear sticky faults on IMU. */
  public abstract void clearStickyFaults();

  /**
   * Set the yaw in degrees.
   *
   * @param yaw Yaw angle in degrees.
   */
  public abstract void setYaw(double yaw);

  /**
   * Fetch the yaw/pitch/roll from the IMU.
   *
   * @param yprArray Array which will be filled with {yaw, pitch, roll} in degrees.
   */
  public abstract void getYawPitchRoll(double[] yprArray);
}
