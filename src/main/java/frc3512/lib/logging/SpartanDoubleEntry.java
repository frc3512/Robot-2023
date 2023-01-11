package frc3512.lib.logging;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

/** Sets up a double value in NetworkTables with the option to be logged */
public class SpartanDoubleEntry {

  private DoubleTopic topic;
  private DoublePublisher pub;
  private DoubleSubscriber sub;
  private DoubleLogEntry log;
  double defaultValue = 0.0;
  boolean logged = false;
  DataLog logInstance = SpartanLogManager.getCurrentLog();

  public SpartanDoubleEntry(String name) {
    this(name, 0.0);
  }

  public SpartanDoubleEntry(String name, double value) {
    this(name, value, false);
  }

  public SpartanDoubleEntry(String name, double value, boolean logged) {
    this.defaultValue = value;
    this.logged = logged;
    topic = SpartanLogManager.getNTInstance().getDoubleTopic(name);
    log = new DoubleLogEntry(logInstance, name);
  }

  public void set(double value) {
    if (pub == null) pub = topic.publish();
    pub.set(value);
    if (SpartanLogManager.isCompetition() && logged) log.append(value);
  }

  public double get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
