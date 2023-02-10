package frc3512.lib.logging;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;

/** Sets up a string value in NetworkTables that is also logged */
public class SpartanStringEntry {

  private StringTopic topic;
  private StringPublisher pub;
  private StringSubscriber sub;
  String defaultValue = "";
  boolean override = false;

  public SpartanStringEntry(String name) {
    this(name, "");
  }

  public SpartanStringEntry(String name, String value) {
    this(name, value, SpartanEntryManager.isTuningMode());
  }

  public SpartanStringEntry(String name, String value, boolean override) {
    this.defaultValue = value;
    this.override = override;
    topic = SpartanEntryManager.getNTInstance().getStringTopic(name);
  }

  public void set(String value) {
    if (pub == null) pub = topic.publish();
    if (override) {
      pub.set(value);
    }
  }

  public String get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
