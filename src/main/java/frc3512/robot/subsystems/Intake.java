package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;
import frc3512.robot.Constants;

public class Intake extends SubsystemBase {
  public enum CurrentGamePiece {
    NONE, CONE, CUBE, INVALID
  }

  CANSparkMax intakeMotor;
  public CurrentGamePiece gamePiece;

  public Intake() {
    intakeMotor =
        new CANSparkMax(
            Constants.IntakeConstants.intakeMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();

    CANSparkMaxUtil.setCANSparkMaxBusUsage(intakeMotor, Usage.kMinimal);
    intakeMotor.setSmartCurrentLimit(40);
    intakeMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);

    intakeMotor.burnFlash();
  }

  public void intakeCone() {
    intakeMotor.set(0.8);
  }

  public void intakeCube() {
    intakeMotor.set(-0.8);
  }

  public void stopIntake() {
    intakeMotor.set(0.0);
  }
}
