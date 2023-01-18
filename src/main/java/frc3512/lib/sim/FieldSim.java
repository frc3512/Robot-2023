package frc3512.lib.sim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldSim {
  private final Field2d field;

  public FieldSim() {
    field = new Field2d();

    SmartDashboard.putData("Field2d", field);
  }

  public void setRobotPose(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      field.setRobotPose(pose);
    }
  }
}
