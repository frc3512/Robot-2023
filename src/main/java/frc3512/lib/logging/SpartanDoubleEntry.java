package frc3512.lib.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;

/** Sets up a double value in NetworkTables that is also logged */
public class SpartanDoubleEntry {

  private DoubleTopic topic;
  private DoublePublisher pub;
  private DoubleSubscriber sub;
  double defaultValue = 0.0;
  boolean override = false;

  public SpartanDoubleEntry(String name) {
    this(name, 0.0);
  }

  public SpartanDoubleEntry(String name, double value) {
    this(name, value, SpartanEntryManager.isTuningMode());
  }

  public SpartanDoubleEntry(String name, double value, boolean override) {
    this.defaultValue = value;
    this.override = override;
    topic = SpartanEntryManager.getNTInstance().getDoubleTopic(name);
  }

  public void set(double value) {
    if (pub == null) pub = topic.publish();
    if (override) {
      pub.set(value);
    }
  }

  public double get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
