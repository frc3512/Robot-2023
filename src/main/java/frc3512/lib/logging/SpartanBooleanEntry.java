package frc3512.lib.logging;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;

/** Sets up a boolean value in NetworkTables. */
public class SpartanBooleanEntry {

  private BooleanTopic topic;
  private BooleanPublisher pub;
  private BooleanSubscriber sub;
  boolean defaultValue = false;
  boolean override = false;

  public SpartanBooleanEntry(String name) {
    this(name, false);
  }

  public SpartanBooleanEntry(String name, boolean value) {
    this(name, value, SpartanEntryManager.isTuningMode());
  }

  public SpartanBooleanEntry(String name, boolean value, boolean override) {
    this.defaultValue = value;
    this.override = override;
    topic = SpartanEntryManager.getNTInstance().getBooleanTopic(name);
  }

  public void set(boolean value) {
    if (pub == null) pub = topic.publish();
    if (override) {
      pub.set(value);
    }
  }

  public boolean get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = false;
    currValue = sub.get();
    return currValue;
  }
}
