package frc3512.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          Constants.ArmConstants.pGain,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              Constants.ArmConstants.maxVelocityRadPerSecond,
              Constants.ArmConstants.maxAccelerationRadPerSecSquared));
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          Constants.ArmConstants.sVolts, Constants.ArmConstants.gVolts,
          Constants.ArmConstants.vVoltSecondPerRad,
              Constants.ArmConstants.aVoltSecondSquaredPerRad);

  private final SpartanDoubleEntry velocityEntry =
      new SpartanDoubleEntry("/Diagnostics/Arm/Velocity");
  private final SpartanDoubleEntry positionEntry =
      new SpartanDoubleEntry("/Diagnostics/Arm/Position");

  public Arm() {
    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    leftArmMotor.setSmartCurrentLimit(80);
    rightArmMotor.setSmartCurrentLimit(80);
    leftArmMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    rightArmMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setInverted(true);

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
      armGroup.setVoltage(output);
    }
  }

  public double getAngle() {
    return armEncoder.getPosition() + Constants.ArmConstants.armOffsetRads;
  }

  public Command runArm(DoubleSupplier joystickValue) {
    return run(
        () -> {
          if (joystickValue.getAsDouble() == 180) {
            armGroup.set(0.3);
          } else if (joystickValue.getAsDouble() == 0) {
            armGroup.set(-0.3);
          } else {
            armGroup.set(0.0);
          }
        });
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    velocityEntry.set(armEncoder.getVelocity());
    positionEntry.set(armEncoder.getPosition());
  }
}
