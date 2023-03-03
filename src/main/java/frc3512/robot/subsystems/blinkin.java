package frc3512.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Robot2023;

public class blinkin extends SubsystemBase {
  private static Spark m_blinkin = null;

  public blinkin(int port) {
    m_blinkin = new Spark(port);
    solid_orange();
  }

  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void solid_orange() {
    set(0.65);
  }

  public void solid_purple() {
    set(0.91);
  }

  public void allianceColor() {
    boolean isRed =
        NetworkTableInstance.getDefault()
            .getTable("FMSInfo")
            .getEntry("IsRedAlliance")
            .getBoolean(true);
    if (isRed == true) {
      Robot2023.m_blinkin.set(-0.01);
      System.out.println("led RED");
    } else {
      Robot2023.m_blinkin.set(0.19);
      System.out.println("led BLUE");
    }
  }
}
