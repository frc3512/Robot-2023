package frc3512.lib.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

/** Sets up a double array value in NetworkTables that is also logged */
public class SpartanPose2dEntry {

  private DoubleArrayTopic topic;
  private DoubleArrayPublisher pub;
  private DoubleArraySubscriber sub;
  private DoubleArrayLogEntry log;
  private Pose2d pose = new Pose2d();
  double[] defaultValue = new double[] {};
  boolean override = false;
  DataLog logInstance = SpartanLogManager.getCurrentLog();

  public SpartanPose2dEntry(String name) {
    this(name, new Pose2d());
  }

  public SpartanPose2dEntry(String name, Pose2d value) {
    this(name, value, SpartanLogManager.isTuningMode());
  }

  public SpartanPose2dEntry(String name, Pose2d value, boolean override) {
    this.pose = value;
    this.override = override;

    double[] converted = new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    this.defaultValue = converted;

    topic = SpartanLogManager.getNTInstance().getDoubleArrayTopic(name);
    log = new DoubleArrayLogEntry(logInstance, name);
  }

  public void set(Pose2d value) {
    if (pub == null) pub = topic.publish();
    double[] converted =
        new double[] {value.getX(), value.getY(), value.getRotation().getDegrees()};
    if (override) {
      pub.set(converted);
      log.append(converted);
    }
  }

  public Pose2d get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    Pose2d converted =
        new Pose2d(
            new Translation2d(currValue[0], currValue[1]), Rotation2d.fromDegrees(currValue[2]));
    return converted;
  }
}
