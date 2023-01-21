package frc3512.lib.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

/** Sets up a double value in NetworkTables that is also logged */
public class SpartanDoubleEntry {

  private DoubleTopic topic;
  private DoublePublisher pub;
  private DoubleSubscriber sub;
  private DoubleLogEntry log;
  double defaultValue = 0.0;
  boolean override = false;
  DataLog logInstance = SpartanLogManager.getCurrentLog();

  public SpartanDoubleEntry(String name) {
    this(name, 0.0);
  }

  public SpartanDoubleEntry(String name, double value) {
    this(name, value, SpartanLogManager.isTuningMode());
  }

  public SpartanDoubleEntry(String name, double value, boolean override) {
    this.defaultValue = value;
    this.override = override;
    topic = SpartanLogManager.getNTInstance().getDoubleTopic(name);
    log = new DoubleLogEntry(logInstance, name);
  }

  public void set(double value) {
    if (pub == null) pub = topic.publish();
    if (override) {
      pub.set(value);
      log.append(value);
    }
  }

  public double get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
