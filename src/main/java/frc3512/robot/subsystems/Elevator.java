package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanBooleanEntry;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  private final CANSparkMax leftElevatorMotor =
      new CANSparkMax(
          Constants.ElevatorConstants.leftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightElevatorMotor =
      new CANSparkMax(
          Constants.ElevatorConstants.rightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final SparkMaxLimitSwitch limitSwitch =
      leftElevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private final RelativeEncoder elevatorEncoder =
      rightElevatorMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 8192);

  private final SlewRateLimiter limiter = new SlewRateLimiter(2.0);

  private SpartanDoubleEntry positionEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Position");
  private SpartanBooleanEntry reverseLimitEntry =
      new SpartanBooleanEntry("/Diagnostics/Elevator/Reverse Limit Reached");

  public Elevator() {
    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(leftElevatorMotor, Usage.kMinimal, true, false, true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(rightElevatorMotor, Usage.kAll, true, false, false);

    leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    leftElevatorMotor.setSmartCurrentLimit(80);
    rightElevatorMotor.setSmartCurrentLimit(80);
    leftElevatorMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    rightElevatorMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);

    rightElevatorMotor.follow(leftElevatorMotor, true);

    leftElevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    elevatorEncoder.setPosition(0.0);
  }

  public Command moveElevator(DoubleSupplier elevator) {
    return run(
        () -> {
          if (elevatorEncoder.getPosition() < 9.0) {
            leftElevatorMotor.set(limiter.calculate(elevator.getAsDouble() * 0.4));
          } else {
            leftElevatorMotor.set(0.0);
          }
        });
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    positionEntry.set(getPosition());
    reverseLimitEntry.set(limitSwitch.isPressed());
  }
}
