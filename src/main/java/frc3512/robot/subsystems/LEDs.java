package frc3512.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.HSV;
import frc3512.robot.Constants;

public class LEDs extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public enum Selection {
    CONE,
    CUBE,
    READY_TO_SCORE,
    MANUAL_MODE,
    ADJUSTING,
  }

  private Selection currSelection;
  private boolean isCone = false;

  public static final int pauseBetween = 15;
  public static final int length = 25;
  public static final double spread = 8;
  public static final double speed = .5;
  private double middleIndex = -pauseBetween;
  public static final HSV hsv = HSV.googleColorPickerHSV(240, 100, 97);

  public LEDs() {
    led = new AddressableLED(Constants.LEDConstants.pwmPort);

    buffer = new AddressableLEDBuffer(Constants.LEDConstants.ledBufferLength);
    led.setLength(buffer.getLength());

    led.setData(buffer);
    led.start();
  }

  public void defaultPattern() {
    for (int i = 0; i < buffer.getLength() / 2 + pauseBetween; i++) {
      int value =
          MathUtil.clamp(
                  (int) ((1 / spread) * (Math.abs(middleIndex - i)) * hsv.v) - length,
                  0,
                  hsv.v - 10)
              + 10;

      if (buffer.getLength() / 2 > i) {
        buffer.setHSV(i, hsv.h, hsv.s, value);
        buffer.setHSV(buffer.getLength() - i - 1, hsv.h, hsv.s, value);
      }
    }
    middleIndex =
        (middleIndex + speed) > buffer.getLength() + pauseBetween
            ? -pauseBetween
            : (middleIndex + speed);

    led.setData(buffer);
  }

  public void setColor(int r, int g, int b) {
    for (var i = 0; i < buffer.getLength(); i++) {
      buffer.setRGB(i, r, g, b);
    }

    led.setData(buffer);
  }

  public void ledStateMachine() {
    switch (currSelection) {
      case CONE:
        setColor(247, 231, 14);
        break;
      case CUBE:
        setColor(106, 0, 220);
        break;
      case READY_TO_SCORE:
        setColor(0, 255, 0);
        break;
      case MANUAL_MODE:
        setColor(249, 150, 2);
        break;
      case ADJUSTING:
        setColor(0, 234, 218);
        break;
    }
  }

  public Command switchColor() {
    return runOnce(
        () -> {
          if (isCone) {
            currSelection = Selection.CUBE;
            isCone = false;
          } else {
            currSelection = Selection.CONE;
            isCone = true;
          }
        });
  }

  public Command goToElementColor() {
    return runOnce(
        () -> {
          if (isCone) {
            currSelection = Selection.CONE;
          } else {
            currSelection = Selection.CUBE;
          }
        });
  }

  public Command selectMode(Selection selection) {
    return runOnce(
        () -> {
          currSelection = selection;
        });
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      defaultPattern();
    } else {
      ledStateMachine();
    }
  }
}
