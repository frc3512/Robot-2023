package frc3512.lib.sim;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class MotorSim {

  private double simPosition;
  private double simVelocity;
  private double currentAngle;
  private double simTurnAngle;
  private double simAngleDiff;

  public void setCurrentAngle(double angle) {
    currentAngle = angle;
  }

  public void updateSimVelocity(SwerveModuleState state) {
    simVelocity = state.speedMetersPerSecond;
    double distance = simVelocity / 50.0;

    simPosition += distance;
  }

  public void updateSimPosition(double angle) {
    if (angle != currentAngle && simTurnAngle == 0) {
      simAngleDiff = angle - currentAngle;
      simTurnAngle = simAngleDiff / 20.0;
    }

    if (simTurnAngle != 0) {
      currentAngle = simTurnAngle;

      if ((Math.abs(angle - currentAngle)) < .1) {
        currentAngle = angle;
        simTurnAngle = 0;
      }
    }
  }

  public double getVelocity() {
    return simVelocity;
  }

  public double getPosition() {
    return simPosition;
  }

  public double getAngle() {
    return currentAngle;
  }
}
