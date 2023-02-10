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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.robot.Constants;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;
  private final AbsoluteEncoder armEncoder;

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

  private final SpartanDoubleEntry velocityEntry, positionEntry;

  public Arm() {
    leftArmMotor = new CANSparkMax(Constants.ArmConstants.leftMotorID, MotorType.kBrushless);
    rightArmMotor = new CANSparkMax(Constants.ArmConstants.rightMotorID, MotorType.kBrushless);
    armEncoder = leftArmMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leftArmMotor.restoreFactoryDefaults();
    rightArmMotor.restoreFactoryDefaults();

    leftArmMotor.setSmartCurrentLimit(80);
    rightArmMotor.setSmartCurrentLimit(80);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);

    rightArmMotor.follow(leftArmMotor, true);

    leftArmMotor.burnFlash();
    rightArmMotor.burnFlash();

    velocityEntry = new SpartanDoubleEntry("/Diagnostics/Arm/Velocity");
    positionEntry = new SpartanDoubleEntry("/Diagnostics/Arm/Position");

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
    return armEncoder.getPosition() + Constants.ArmConstants.armOffsetRads;
  }

  public Command runArm(DoubleSupplier joystickValue) {
    return run(
        () -> {
          leftArmMotor.set(joystickValue.getAsDouble() * 0.8);
        });
  }

  @Override
  public void periodic() {
    controllerPeriodic();
    velocityEntry.set(armEncoder.getVelocity());
    positionEntry.set(armEncoder.getPosition());
  }
}
