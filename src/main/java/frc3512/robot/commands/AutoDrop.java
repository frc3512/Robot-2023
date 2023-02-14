package frc3512.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class AutoDrop {
  public static String getInput(CommandXboxController xboxController, String outtakeLevel) {
    final double Ypos = xboxController.getLeftY();
    final double Xpos = xboxController.getLeftX();
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

  public static String getJoystick(CommandJoystick joystick) {
    var rawJoystick = joystick.getHID();
    double rawDirection = rawJoystick.getRawAxis(5);
    String direction = "";
    String result = "";
    String level = "";
    String intake = "";
    int outtake = 0;

    if (rawJoystick.getRawButtonPressed(11)) {
      level = "L1";
    } else if (rawJoystick.getRawButtonPressed(9)) {
      level = "L2";
    } else if (rawJoystick.getRawButtonPressed(7)) {
      level = "L3";
    } else {
      level = "";
    }

    if (rawJoystick.getRawButtonPressed(3)) {
      intake = "Cube Ground";
    } else if (rawJoystick.getRawButtonPressed(4)) {
      intake = "Cone Ground";
    } else if (rawJoystick.getRawButtonPressed(5)) {
      intake = "Cube HP";
    } else if (rawJoystick.getRawButtonPressed(6)) {
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

    if (rawJoystick.getRawButton(1)) {
      outtake = 1;
    } else {
      outtake = 0;
    }

    result = level + " " + intake + " " + outtake + " " + direction;
    return result;
  }
}
