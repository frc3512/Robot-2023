package frc3512.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  public enum LEDColor {
    GREEN,
    RED,
    BLUE,
    YELLOW,
    PURPLE,
    TEAL,
    WHITE,
    OFF
  }

  private final Solenoid green = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  private final Solenoid blue = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  private final Solenoid red = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

  public Command switchLEDMode(LEDColor color) {
    return runOnce(
        () -> {
          if (green.get()) {
            green.set(false);
          } else {
            green.set(true);
          }

          if (blue.get()) {
            blue.set(false);
          } else {
            blue.set(true);
          }

          if (red.get()) {
            red.set(false);
          } else {
            red.set(true);
          }
        });
  }
}
