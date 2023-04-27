package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor =
      new CANSparkMax(
          Constants.IntakeConstants.intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);

  public enum IntakeDirection {
    INTAKE,
    HALF_SPEED,
    OUTTAKE,
    STOP
  }

  private IntakeDirection direction = IntakeDirection.STOP;

  public Intake() {
    intakeMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.currentLimit);
    intakeMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);

    intakeMotor.burnFlash();
  }

  public Command intakeGamePiece() {
    return runOnce(
        () -> {
          direction = IntakeDirection.INTAKE;
        });
  }

  public Command outtakeGamePiece() {
    return runOnce(
        () -> {
          direction = IntakeDirection.OUTTAKE;
        });
  }

  public Command halfOuttakeGamePiece() {
    return runOnce(
        () -> {
          direction = IntakeDirection.HALF_SPEED;
        });
  }

  public Command stopIntake() {
    return runOnce(
        () -> {
          direction = IntakeDirection.STOP;
        });
  }

  @Override
  public void periodic() {
    if (direction == IntakeDirection.INTAKE) {
      intakeMotor.set(Constants.IntakeConstants.motorSpeed);
    } else if (direction == IntakeDirection.OUTTAKE) {
      intakeMotor.set(-Constants.IntakeConstants.motorSpeed);
    } else if (direction == IntakeDirection.HALF_SPEED) {
      intakeMotor.set(Constants.IntakeConstants.motorSpeed * 0.5);
    } else if (direction == IntakeDirection.STOP) {
      intakeMotor.set(0.0);
    }
  }
}
