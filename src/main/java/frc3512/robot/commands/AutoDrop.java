package frc3512.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.Constants;

public class AutoDrop {
    private final static CommandXboxController xbox =
        new CommandXboxController(Constants.OperatorConstants.appendageControllerPort);
    
    private final static Joystick logitech = 
        new Joystick(Constants.OperatorConstants.appendageControllerPort); 


  public static String getInput(String outtakeLevel) {
    final double Ypos = xbox.getLeftY();
    final double Xpos = xbox.getLeftX();
    String result = "";

    if (outtakeLevel == "L1") {
      if (Xpos > 0.5) {
        result = "Hybrid Right";
      } else if (Xpos < -0.5) {
        result = "Hybrid Left";
      } else if (Ypos < -0.5) {
        result = "Hybrid Middle";
      }
    } else if (outtakeLevel == "L2") {
      if (Xpos > 0.5) {
        result = "Cone Right L2";
      } else if (Xpos < -0.5) {
        result = "Cone Left L2";
      } else if (Ypos < -0.5) {
        result = "Cube L2";
      }
    } else if (outtakeLevel == "L3") {
      if (Xpos > 0.5) {
        result = "Cone Right L3";
      } else if (Xpos < -0.5) {
        result = "Cone Left L3";
      } else if (Ypos < -0.5) {
        result = "Cube L3";
      }
    } else {
      result = null;
    }
    return result;
  }

  public static String getJoystick() {
    double rawDirection = logitech.getRawAxis(5);
    String direction = "";
    String result = "";
    String level = "";
    String intake = "";
    int outtake = 0;

    if (logitech.getRawButtonPressed(11)) {
      level = "L1";
    } else if (logitech.getRawButtonPressed(9)) {
      level = "L2";
    } else if (logitech.getRawButtonPressed(7)) {
      level = "L3";
    } else {
      level = "";
    }

    if (logitech.getRawButtonPressed(3)) {
      intake = "Cube Ground";
    } else if (logitech.getRawButtonPressed(4)) {
      intake = "Cone Ground";
    } else if (logitech.getRawButtonPressed(5)) {
      intake = "Cube HP";
    } else if (logitech.getRawButtonPressed(6)) {
      intake = "Cone HP";
    } else {
      intake = "";
    }

    if (rawDirection > 0.5) {
      direction = "right";
    } else if (rawDirection < -0.5) {
      direction = "left";
    } else {
      direction = "";
    }

    if (logitech.getRawButton(1)) {
      outtake = 1;
    } else {
      outtake = 0;
    }

    result = level + " " + intake + " " + outtake + " " + direction;
    return result;
  }
}
