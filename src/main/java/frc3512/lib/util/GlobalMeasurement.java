package frc3512.lib.util;

import edu.wpi.first.math.geometry.Pose2d;

public class GlobalMeasurement {

  public final Pose2d pose;
  public final double timestampSeconds;
  public final double ambiguity;
  public final double yaw;
  public final double distance;

  public GlobalMeasurement(
      Pose2d pose, double timestampSeconds, double ambiguity, double yaw, double distance) {
    this.pose = pose;
    this.timestampSeconds = timestampSeconds;
    this.ambiguity = ambiguity;
    this.yaw = yaw;
    this.distance = distance;
  }
}
