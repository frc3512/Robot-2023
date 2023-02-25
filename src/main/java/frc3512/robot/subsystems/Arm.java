package frc3512.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  private final CANSparkMax leftArmMotor =
      new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor =
      new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);
  private final AbsoluteEncoder armEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private boolean isClosedLoop;
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.ArmConstants.pGain,
          Constants.ArmConstants.iGain,
          Constants.ArmConstants.dGain,
          new TrapezoidProfile.Constraints(
              Constants.ArmConstants.maxVelocityRadPerSecond,
              Constants.ArmConstants.maxAccelerationRadPerSecSquared));
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          Constants.ArmConstants.sVolts, Constants.ArmConstants.gVolts,
          Constants.ArmConstants.vVoltSecondPerRad,
              Constants.ArmConstants.aVoltSecondSquaredPerRad);

  private final SlewRateLimiter limiter = new SlewRateLimiter(4.0);

  private final SpartanDoubleEntry positionEntry =
      new SpartanDoubleEntry("/Diagnostics/Arm/Position");

  public Arm() {
    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(leftArmMotor, Usage.kAll, true, false, true);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(rightArmMotor, Usage.kMinimal, true, false, false);

    leftArmMotor.setSmartCurrentLimit(40);
    rightArmMotor.setSmartCurrentLimit(40);
    leftArmMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    rightArmMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.follow(leftArmMotor, true);

    armEncoder.setPositionConversionFactor(Math.PI * 2.0);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();

    disable();
  }

  public void enable() {
    isClosedLoop = true;
    controller.reset(getAngle());
  }

  public void disable() {
    isClosedLoop = false;
    controller.setGoal(new State());
  }

  public boolean isClosedLoopEnabled() {
    return isClosedLoop;
  }

  public Command setGoal(TrapezoidProfile.State state) {
    return runOnce(() -> controller.setGoal(state));
  }

  public void controllerPeriodic() {
    if (isClosedLoop) {
      double output =
          controller.calculate(getAngle(), controller.getSetpoint())
              + feedforward.calculate(getAngle(), getAngle());
      leftArmMotor.setVoltage(output);
    }
  }

  public double getAngle() {
    return armEncoder.getPosition();
  }

  public Command runArm(DoubleSupplier joystickValue) {
    return run(
        () -> {
          if (!isClosedLoop) {
            if (joystickValue.getAsDouble() == 180) {
              leftArmMotor.set(limiter.calculate(0.3));
            } else if (joystickValue.getAsDouble() == 0) {
              leftArmMotor.set(limiter.calculate(-0.3));
            } else {
              leftArmMotor.set(limiter.calculate(0.0));
            }
          }
        });
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    positionEntry.set(getAngle());
  }
}
