package frc3512.lib.sim;

import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.util.Units;

public class GyroSim {
  private final BasePigeonSimCollection gyroSim;
  private double yawSim = 0.0;

  public GyroSim(WPI_Pigeon2 gyro) {
    gyroSim = gyro.getSimCollection();
  }

  public void setYaw(double heading) {
    yawSim += heading * 0.02;
    gyroSim.setRawHeading(-Units.radiansToDegrees(yawSim));
  }
}
