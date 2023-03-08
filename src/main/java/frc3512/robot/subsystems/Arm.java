package frc3512.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanBooleanEntry;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.robot.Constants;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  private final CANSparkMax leftArmMotor =
      new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
  private final CANSparkMax rightArmMotor =
      new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);
  private final MotorControllerGroup armGroup =
      new MotorControllerGroup(leftArmMotor, rightArmMotor);
  private final AbsoluteEncoder armEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private boolean isClosedLoop;
  private TrapezoidProfile.State goal;
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.ArmConstants.pGain,
          Constants.ArmConstants.iGain,
          Constants.ArmConstants.dGain,
          new TrapezoidProfile.Constraints(
              Constants.ArmConstants.maxVelocityRadPerSecond,
              Constants.ArmConstants.maxAccelerationRadPerSecSquared));
  private final SlewRateLimiter limiter = new SlewRateLimiter(4.0);

  private final SpartanDoubleEntry positionEntry =
      new SpartanDoubleEntry("/Diagnostics/Arm/Position");
  private SpartanDoubleEntry currGoalEntry =
      new SpartanDoubleEntry("/Diagnostics/Arm/Current Goal");
  private SpartanBooleanEntry goalEntry = new SpartanBooleanEntry("/Diagnostics/Arm/Goal Reached");

  public Arm() {
    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    leftArmMotor.setSmartCurrentLimit(Constants.ArmConstants.currentLimit);
    rightArmMotor.setSmartCurrentLimit(Constants.ArmConstants.currentLimit);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setInverted(true);
    armGroup.setInverted(true);

    armEncoder.setPositionConversionFactor(Constants.ArmConstants.positionConversionFactor);
    armEncoder.setInverted(true);
    armEncoder.setZeroOffset(Constants.ArmConstants.armOffset);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();

    armGroup.set(0.0);

    enable();
  }

  public void enable() {
    isClosedLoop = true;
    controller.reset(getAngle());
    if (DriverStation.isTeleopEnabled()) {
      goal = new State(2.62, 0.0);
    }
  }

  public void disable() {
    isClosedLoop = false;
    controller.setGoal(new State());
  }

  public CommandBase setGoal(TrapezoidProfile.State state) {
    return runOnce(
        () -> {
          goal = state;
        }).until(controller::atGoal);
  }

  public CommandBase runArm(DoubleSupplier joystickValue) {
    return run(
        () -> {
          if (!isClosedLoopEnabled()) {
            if (joystickValue.getAsDouble() == 0) {
              armGroup.set(limiter.calculate(Constants.ArmConstants.teleopSpeed));
            } else if (joystickValue.getAsDouble() == 180) {
              armGroup.set(limiter.calculate(-Constants.ArmConstants.teleopSpeed));
            } else {
              armGroup.set(limiter.calculate(0.0));
            }
          }
        });
  }

  public void controllerPeriodic() {
    if (isClosedLoopEnabled()) {
      if (goal != null) {
        controller.setGoal(
            new TrapezoidProfile.State(
                MathUtil.clamp(
                    goal.position,
                    Constants.ArmConstants.minAngle,
                    Constants.ArmConstants.maxAngle),
                goal.velocity));
      } else {
        controller.setGoal(new State(2.62, 0.0));
      }

      armGroup.setVoltage(controller.calculate(getAngle(), controller.getGoal()));
    }
  }

  public boolean isClosedLoopEnabled() {
    return isClosedLoop;
  }

  public double getAngle() {
    return armEncoder.getPosition();
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    positionEntry.set(getAngle());
    currGoalEntry.set(controller.getGoal().position);
    goalEntry.set(controller.atGoal());
  }
}
