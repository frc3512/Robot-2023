package frc3512.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanBooleanEntry;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  private final CANSparkMax leftElevatorMotor;
  private final CANSparkMax rightElevatorMotor;
  private final AbsoluteEncoder elevatorEncoder;

  private SpartanDoubleEntry velocityEntry, positionEntry;
  private SpartanBooleanEntry reverseLimitEntry;

  public Elevator() {
    leftElevatorMotor =
        new CANSparkMax(
            Constants.ElevatorConstants.leftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    rightElevatorMotor =
        new CANSparkMax(
            Constants.ElevatorConstants.rightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();
    elevatorEncoder = leftElevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    leftElevatorMotor.setSmartCurrentLimit(80);
    rightElevatorMotor.setSmartCurrentLimit(80); 

    rightElevatorMotor.follow(leftElevatorMotor, true);

    leftElevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    velocityEntry = new SpartanDoubleEntry("/Diagnostics/Elevator/Velocity");
    positionEntry = new SpartanDoubleEntry("/Diagnostics/Elevator/Position");
    reverseLimitEntry = new SpartanBooleanEntry("/Diagnostics/Elevator/Reverse Limit Reached");
  }

  public Command moveElevator(DoubleSupplier elevator) {
    return run(
        () -> {
          leftElevatorMotor.set((elevator.getAsDouble() - 0.3) * 0.3);
        });
  }

  @Override
  public void periodic() {
    velocityEntry.set(elevatorEncoder.getVelocity());
    positionEntry.set(elevatorEncoder.getPosition());
  }
}
