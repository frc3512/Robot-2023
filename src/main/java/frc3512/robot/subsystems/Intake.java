package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax intakeMotor =
      new CANSparkMax(
          Constants.IntakeConstants.intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
  private SpartanDoubleEntry currentEntry =
      new SpartanDoubleEntry("/Diagnostics/Intake/Motor Current");

  public Intake() {
    intakeMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal);
    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);

    intakeMotor.burnFlash();
  }

  public Command intakeGamePiece() {
    return run(
        () -> {
          intakeMotor.set(0.5);
        });
  }

  public Command outtakeGamePiece() {
    return run(
        () -> {
          intakeMotor.set(-0.5);
        });
  }

  public Command stopIntake() {
    return run(
        () -> {
          intakeMotor.set(0.0);
        });
  }

  public Command applyCurrentSensing(double normalSpeed, double stallSpeed) {
    return run(
        () -> {
          if (intakeMotor.getOutputCurrent() >= Constants.IntakeConstants.intakeCurrentThreshold) {
            intakeMotor.set(stallSpeed);
          } else {
            intakeMotor.set(normalSpeed);
          }
        });
  }

  @Override
  public void periodic() {
    currentEntry.set(intakeMotor.getOutputCurrent());
  }
}
