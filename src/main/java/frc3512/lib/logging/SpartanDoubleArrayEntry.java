package frc3512.lib.logging;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;

/** Sets up a double array value in NetworkTables with the option to be logged */
public class SpartanDoubleArrayEntry {

  private DoubleArrayTopic topic;
  private DoubleArrayPublisher pub;
  private DoubleArraySubscriber sub;
  private DoubleArrayLogEntry log;
  double[] defaultValue = new double[] {};
  boolean logged = false;
  DataLog logInstance = SpartanLogManager.getCurrentLog();

  public SpartanDoubleArrayEntry(String name) {
    this(name, new double[] {});
  }

  public SpartanDoubleArrayEntry(String name, double[] value) {
    this(name, value, false);
  }

  public SpartanDoubleArrayEntry(String name, double[] value, boolean logged) {
    this.defaultValue = value;
    this.logged = logged;
    topic = SpartanLogManager.getNTInstance().getDoubleArrayTopic(name);
    log = new DoubleArrayLogEntry(logInstance, name);
  }

  public void set(double[] value) {
    if (pub == null) pub = topic.publish();
    pub.set(value);
    if (SpartanLogManager.isCompetition() && logged) log.append(value);
  }

  public double[] get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
