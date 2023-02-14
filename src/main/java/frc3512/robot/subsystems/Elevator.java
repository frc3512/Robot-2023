package frc3512.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private SparkMaxLimitSwitch m_forwardLimit;
  private SparkMaxLimitSwitch m_reverseLimit;

  private MechanismLigament2d m_elevator;
  Mechanism2d mech = new Mechanism2d(3, 3);
  MechanismRoot2d root = mech.getRoot("Elevator", 2, 0);

  private SpartanDoubleEntry velocityEntry, positionEntry;
  private SpartanBooleanEntry forwardLimitEntry, reverseLimitEntry;

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

    leftElevatorMotor.setSmartCurrentLimit(40);
    rightElevatorMotor.setSmartCurrentLimit(40);

    rightElevatorMotor.follow(leftElevatorMotor);

    m_forwardLimit =
        leftElevatorMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
    m_reverseLimit =
        rightElevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

    m_forwardLimit.enableLimitSwitch(true);
    m_reverseLimit.enableLimitSwitch(true);

    leftElevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    velocityEntry = new SpartanDoubleEntry("/Diagnostics/Elevator/Velocity");
    positionEntry = new SpartanDoubleEntry("/Diagnostics/Elevator/Position");
    forwardLimitEntry = new SpartanBooleanEntry("/Diagnostics/Elevator/Forward Limit Reached");
    reverseLimitEntry = new SpartanBooleanEntry("/Diagnostics/Elevator/Reverse Limit Reached");

    m_elevator =
        root.append(
            new MechanismLigament2d(
                "Elevator", Constants.ElevatorConstants.simElevatorMinimumLength, 90));
    SmartDashboard.putData("Elevator Scoring Mechanism", mech);
  }

  public Command moveElevator(DoubleSupplier elevator) {
    return this.run(
        () -> {
          leftElevatorMotor.set(elevator.getAsDouble());
        });
  }

  @Override
  public void periodic() {
    velocityEntry.set(elevatorEncoder.getVelocity());
    positionEntry.set(elevatorEncoder.getPosition());
    forwardLimitEntry.set(m_forwardLimit.isPressed());
    reverseLimitEntry.set(m_reverseLimit.isPressed());
  }

  @Override
  public void simulationPeriodic() {
    m_elevator.setLength(
        Constants.ElevatorConstants.simElevatorMinimumLength + elevatorEncoder.getPosition());
  }
}
