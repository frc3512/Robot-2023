package frc3512.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;

public class LEDs extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  public enum Selection {
    CONE,
    CUBE,
    READY_TO_SCORE,
    MANUAL_MODE,
    RED_ALLIANCE,
    BLUE_ALLIANCE,
    DEFAULT
  }

  private Selection currSelection;
  private boolean isCone = false;

  public LEDs() {
    m_led = new AddressableLED(Constants.LEDConstants.pwmPort);

    m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.ledBufferLength);
    m_led.setLength(m_ledBuffer.getLength());

    m_led.setData(m_ledBuffer);
    m_led.start();

    currSelection = Selection.DEFAULT;
  }

  public void setColor(int r, int g, int b) {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_led.setData(m_ledBuffer);
  }

  public void ledStateMachine() {
    switch (currSelection) {
      case DEFAULT:
        setColor(255, 255, 255);
        break;
      case CONE:
        setColor(247, 231, 14);
        break;
      case CUBE:
        setColor(106, 0, 220);
        break;
      case READY_TO_SCORE:
        setColor(0, 255, 0);
        break;
      case RED_ALLIANCE:
        setColor(255, 0, 0);
        break;
      case BLUE_ALLIANCE:
        setColor(0, 0, 255);
        break;
      case MANUAL_MODE:
        setColor(249, 150, 2);
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
      currSelection = Selection.DEFAULT;
    } else if (DriverStation.isAutonomous()) {
      if (DriverStation.getAlliance() == Alliance.Red) {
        currSelection = Selection.RED_ALLIANCE;
      } else if (DriverStation.getAlliance() == Alliance.Blue) {
        currSelection = Selection.BLUE_ALLIANCE;
      } else {
        currSelection = Selection.DEFAULT;
      }
    }

    ledStateMachine();
  }
}
