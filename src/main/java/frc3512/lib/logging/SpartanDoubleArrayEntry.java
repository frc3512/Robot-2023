package frc3512.lib.logging;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;

/** Sets up a double array value in NetworkTables that is also logged */
public class SpartanDoubleArrayEntry {

  private DoubleArrayTopic topic;
  private DoubleArrayPublisher pub;
  private DoubleArraySubscriber sub;
  double[] defaultValue = new double[] {};
  boolean override = false;

  public SpartanDoubleArrayEntry(String name) {
    this(name, new double[] {});
  }

  public SpartanDoubleArrayEntry(String name, double[] value) {
    this(name, value, SpartanEntryManager.isTuningMode());
  }

  public SpartanDoubleArrayEntry(String name, double[] value, boolean override) {
    this.defaultValue = value;
    this.override = override;
    topic = SpartanEntryManager.getNTInstance().getDoubleArrayTopic(name);
  }

  public void set(double[] value) {
    if (pub == null) pub = topic.publish();
    if (override) {
      pub.set(value);
    }
  }

  public double[] get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
