package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import frc3512.lib.util.CANSparkMaxUtil;
import frc3512.lib.util.CANSparkMaxUtil.Usage;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;

public class Intake extends SubsystemBase{
    CANSparkMax m_intakeMotor =
      new CANSparkMax(Constants.IntakeConstants.MotorID, CANSparkMaxLowLevel.MotorType.kBrushless);


    public Intake() {
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_intakeMotor, Usage.kMinimal);
        m_intakeMotor.setSmartCurrentLimit(80);
        m_intakeMotor.enableVoltageCompensation(Constants.GeneralConstants.voltageComp);
        m_intakeMotor.burnFlash();
    }

    public void intakeCone() {
        m_intakeMotor.set(0.8);
    }
    
    public void intakeCube() {
        m_intakeMotor.set(-0.8);
    }
    
    public void stopIntake() {
        m_intakeMotor.set(0.0);
    }
}
