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

  public Intake() {
    intakeMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal);
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(Constants.IntakeConstants.currentLimit);
    intakeMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);

    intakeMotor.burnFlash();
  }

  public Command intakeGamePiece() {
    return run(
        () -> {
          intakeMotor.set(Constants.IntakeConstants.motorSpeed);
        });
  }

  public Command outtakeGamePiece() {
    return run(
        () -> {
          intakeMotor.set(-Constants.IntakeConstants.motorSpeed);
        });
  }

  public Command halfOuttakeGamePiece() {
    return run(
        () -> {
          intakeMotor.set(Constants.IntakeConstants.motorSpeed * 0.5);
        });
  }

  public Command stopIntake() {
    return run(
        () -> {
          intakeMotor.set(0.0);
        });
  }
}
