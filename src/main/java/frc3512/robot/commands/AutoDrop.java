package frc3512.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc3512.robot.Constants;

public class AutoDrop {
    private final static CommandXboxController driver =
      new CommandXboxController(Constants.OperatorConstants.xboxController2Port);
    
    private static String getDirection() {
        final double Ypos = driver.getLeftY();
        final double Xpos = driver.getLeftX();

        if (Ypos < -0.5) {
            return "up"; // Outtake to L2
        } else if (Ypos > 0.5) {
            return "down"; // Intake from Ground
        } else if (Xpos > 0.5) {
            return "right"; // Outake to L3
        } else if (Xpos < -0.5) {
            return "left"; // Outake to Hybrid zone
        }
        return null; 
    }
    
    public static void dropper(String bumper) {
        String direction = getDirection();
        if (bumper == "left") { // Cube 
            if (direction == "up") {
                // Outtake cube to L2
            } else if (direction == "down") {
                // Intake Cube
            } else if (direction == "left") {
                // Outtake cube to Hybrid
            } else if (direction == "right") {
                // Outtake cube to L3
            } 
        } else if (bumper == "right") { // Cone intake
            if (direction == "up") {
                // Outtake cone to L2
            } else if (direction == "down") {
                // Intake Cone
            } else if (direction == "left") {
                // Outtake cone to Hybrid
            } else if (direction == "right") {
                // Outtake cone to L3
            } 
        }
    }
}
