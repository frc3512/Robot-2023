package frc3512.lib.logging;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;

/** Sets up a boolean value in NetworkTables that is also logged */
public class SpartanBooleanEntry {

  private BooleanTopic topic;
  private BooleanPublisher pub;
  private BooleanSubscriber sub;
  private BooleanLogEntry log;
  boolean defaultValue = false;
  boolean override = false;
  DataLog logInstance = SpartanLogManager.getCurrentLog();

  public SpartanBooleanEntry(String name) {
    this(name, false);
  }

  public SpartanBooleanEntry(String name, boolean value) {
    this(name, value, SpartanLogManager.isTuningMode());
  }

  public SpartanBooleanEntry(String name, boolean value, boolean override) {
    this.defaultValue = value;
    this.override = override;
    topic = SpartanLogManager.getNTInstance().getBooleanTopic(name);
    log = new BooleanLogEntry(logInstance, name);
  }

  public void set(boolean value) {
    if (pub == null) pub = topic.publish();
    if (override) {
      pub.set(value);
      log.append(value);
    }
  }

  public boolean get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = false;
    currValue = sub.get();
    return currValue;
  }
}
