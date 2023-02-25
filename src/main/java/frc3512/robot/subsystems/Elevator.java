package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxLimitSwitch;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
  private final SparkMaxLimitSwitch limitSwitch =
      leftElevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private final Encoder elevatorEncoder =
      new Encoder(Constants.ElevatorConstants.encoderA, Constants.ElevatorConstants.encoderB);

  private boolean isClosedLoop;
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.ElevatorConstants.pGain,
          Constants.ElevatorConstants.iGain,
          Constants.ElevatorConstants.dGain,
          new TrapezoidProfile.Constraints(
              Constants.ElevatorConstants.maxVelocityMeterPerSecond,
              Constants.ElevatorConstants.maxAccelerationMeterPerSecondSquared));

  private final SlewRateLimiter limiter = new SlewRateLimiter(2.0);

  private SpartanDoubleEntry positionEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Position");
  private SpartanDoubleEntry currGoalEntry =
      new SpartanDoubleEntry("/Diagnostics/Elevator/Current Goal");
  private SpartanBooleanEntry goalEntry =
      new SpartanBooleanEntry("/Diagnostics/Elevator/Goal Reached");
  private SpartanBooleanEntry reverseLimitEntry =
      new SpartanBooleanEntry("/Diagnostics/Elevator/Bottom Limit");

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

    elevatorEncoder.setDistancePerPulse((Math.PI * 2.0 * Units.inchesToMeters(1.5)) / 8192);
    elevatorEncoder.setReverseDirection(true);

    leftElevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    elevatorGroup.set(0.0);
    elevatorEncoder.reset();

    enable();
  }

  public void enable() {
    isClosedLoop = true;
    controller.reset(getPosition());
  }

  public void disable() {
    isClosedLoop = false;
    controller.setGoal(new State());
  }

  public CommandBase setGoal(TrapezoidProfile.State state) {
    return runOnce(
            () -> {
              controller.setGoal(state);
            })
        .andThen(run(() -> elevatorGroup.setVoltage(controller.calculate(getPosition()))))
        .until(() -> controller.atGoal())
        .finallyDo(interrupted -> elevatorGroup.set(0.0));
  }

  public CommandBase moveElevator(DoubleSupplier elevator) {
    return run(
        () -> {
          if (!isClosedLoop) {
            elevatorGroup.set(limiter.calculate(elevator.getAsDouble() * 0.4));
          }
        });
  }

  public boolean isClosedLoopEnabled() {
    return isClosedLoop;
  }

  public double getPosition() {
    return elevatorEncoder.getDistance();
  }

  @Override
  public void periodic() {
    positionEntry.set(getPosition());
    currGoalEntry.set(controller.getGoal().position);
    goalEntry.set(controller.atGoal());
    reverseLimitEntry.set(limitSwitch.isPressed());
  }
}
