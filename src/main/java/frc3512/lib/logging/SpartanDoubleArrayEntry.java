package frc3512.lib.logging;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

/** Sets up a double array value in NetworkTables that is also logged */
public class SpartanDoubleArrayEntry {

  private DoubleArrayTopic topic;
  private DoubleArrayPublisher pub;
  private DoubleArraySubscriber sub;
  private DoubleArrayLogEntry log;
  double[] defaultValue = new double[] {};
  boolean override = false;
  DataLog logInstance = SpartanLogManager.getCurrentLog();

  public SpartanDoubleArrayEntry(String name) {
    this(name, new double[] {});
  }

  public SpartanDoubleArrayEntry(String name, double[] value) {
    this(name, value, SpartanLogManager.isTuningMode());
  }

  public SpartanDoubleArrayEntry(String name, double[] value, boolean override) {
    this.defaultValue = value;
    this.override = override;
    topic = SpartanLogManager.getNTInstance().getDoubleArrayTopic(name);
    log = new DoubleArrayLogEntry(logInstance, name);
  }

  public void set(double[] value) {
    if (pub == null) pub = topic.publish();
    if (override) {
      pub.set(value);
      log.append(value);
    }
  }

  public double[] get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
