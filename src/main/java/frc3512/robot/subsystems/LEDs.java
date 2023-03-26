package frc3512.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final Solenoid leds = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  public Command switchLEDMode() {
    return runOnce(
        () -> {
          if (leds.get()) {
            leds.set(false);
          } else {
            leds.set(true);
          }
        });
  }
}