package frc3512.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanBooleanEntry;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.robot.Constants;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase {
  private final CANSparkMax leftElevatorMotor =
      new CANSparkMax(
          Constants.ElevatorConstants.leftMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax rightElevatorMotor =
      new CANSparkMax(
          Constants.ElevatorConstants.rightMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final MotorControllerGroup elevatorGroup =
      new MotorControllerGroup(leftElevatorMotor, rightElevatorMotor);
  private final AbsoluteEncoder elevatorEncoder =
      leftElevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private final SparkMaxLimitSwitch reverseLimit =
      rightElevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private boolean isClosedLoop;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Constants.ElevatorConstants.maxVelocity, Constants.ElevatorConstants.maxAcceleration);
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.ElevatorConstants.pGain,
          Constants.ElevatorConstants.iGain,
          Constants.ElevatorConstants.dGain,
          constraints);
  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          Constants.ElevatorConstants.sVolts,
          Constants.ElevatorConstants.gVolts,
          Constants.ElevatorConstants.vVoltSecondPerMeter,
          Constants.ElevatorConstants.aVoltSecondsSquaredPerMeter);

  private final SpartanDoubleEntry velocityEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Velocity");
  private final SpartanDoubleEntry positionEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Position");
  private final SpartanBooleanEntry reverseLimitEntry =
      new SpartanBooleanEntry("/Diagnostics/Elevator/Reverse Limit Reached");

  public Elevator() {
    leftElevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();

    leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    leftElevatorMotor.setSmartCurrentLimit(80);
    rightElevatorMotor.setSmartCurrentLimit(80);
    leftElevatorMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    rightElevatorMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    rightElevatorMotor.setInverted(true);
    reverseLimit.enableLimitSwitch(false);

    leftElevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    disable();
  }

  public void enable() {
    isClosedLoop = true;
    controller.reset(getPosition());
  }

  public void disable() {
    isClosedLoop = false;
    controller.setGoal(0.0);
  }

  public boolean isClosedLoopEnabled() {
    return isClosedLoop;
  }

  public Command setGoal(double heightMeters) {
    return runOnce(
        () -> {
          controller.setGoal(heightMeters);
        });
  }

  public void controllerPeriodic() {
    if (isClosedLoop) {
      double output =
          controller.calculate(elevatorEncoder.getPosition())
              + feedforward.calculate(controller.getSetpoint().velocity);
      elevatorGroup.setVoltage(output);
    }
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public Command moveElevator(DoubleSupplier elevator) {
    return run(
        () -> {
          elevatorGroup.set((elevator.getAsDouble() - 0.3) * 0.3);
        });
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    velocityEntry.set(elevatorEncoder.getVelocity());
    positionEntry.set(elevatorEncoder.getPosition());
    reverseLimitEntry.set(reverseLimit.isPressed());
  }
}
