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
    return run(
        () -> {
          if (color == LEDColor.WHITE) {
            green.set(true);
            blue.set(true);
            red.set(true);
          } else if (color == LEDColor.GREEN) {
            green.set(true);
            blue.set(false);
            red.set(false);
          } else if (color == LEDColor.RED) {
            green.set(false);
            blue.set(false);
            red.set(true);
          } else if (color == LEDColor.BLUE) {
            green.set(false);
            blue.set(true);
            red.set(false);
          } else if (color == LEDColor.YELLOW) {
            green.set(true);
            blue.set(false);
            red.set(true);
          } else if (color == LEDColor.PURPLE) {
            green.set(false);
            blue.set(true);
            red.set(true);
          } else if (color == LEDColor.TEAL) {
            green.set(true);
            blue.set(true);
            red.set(false);
          } else {
            green.set(false);
            blue.set(false);
            red.set(false);
          }
        });
  }
}
