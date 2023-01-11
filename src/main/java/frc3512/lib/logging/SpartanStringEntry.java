package frc3512.lib.logging;

import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;

/** Sets up a string value in NetworkTables with the option to be logged */
public class SpartanStringEntry {

  private StringTopic topic;
  private StringPublisher pub;
  private StringSubscriber sub;
  private StringLogEntry log;
  String defaultValue = "";
  boolean logged = false;
  DataLog logInstance = SpartanLogManager.getCurrentLog();

  public SpartanStringEntry(String name) {
    this(name, "");
  }

  public SpartanStringEntry(String name, String value) {
    this(name, value, false);
  }

  public SpartanStringEntry(String name, String value, boolean logged) {
    this.defaultValue = value;
    this.logged = logged;
    topic = SpartanLogManager.getNTInstance().getStringTopic(name);
    log = new StringLogEntry(logInstance, name);
  }

  public void set(String value) {
    if (pub == null) pub = topic.publish();
    pub.set(value);
    if (SpartanLogManager.isCompetition() && logged) log.append(value);
  }

  public String get() {
    if (sub == null) sub = topic.subscribe(defaultValue);
    var currValue = sub.get();
    return currValue;
  }
}
