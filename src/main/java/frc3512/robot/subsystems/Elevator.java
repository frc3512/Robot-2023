package frc3512.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants;
import frc3512.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private static final double kMetersPerPulse = Constants.ElevatorConstants.distancePerpulse;
    private static final double kElevatorMinimumLength = 0.5;

    private final CANSparkMax m_elevatorMotorM;
    private final CANSparkMax m_elevatorMotorS;
    private final Encoder m_elevatorEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    private SparkMaxLimitSwitch m_forwardLimit;
    private SparkMaxLimitSwitch m_reverseLimit;

    private MechanismLigament2d m_elevator;

    Mechanism2d mech = new Mechanism2d(3,3);
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);

    public Elevator() {
        m_elevatorMotorM = new CANSparkMax(Constants.ElevatorConstants.MotorBID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_elevatorMotorS = new CANSparkMax(Constants.ElevatorConstants.MotorAID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_elevatorMotorM.restoreFactoryDefaults();
        m_elevatorMotorS.restoreFactoryDefaults();
        m_elevatorEncoder = new Encoder(ElevatorConstants.EncoderA, ElevatorConstants.EncoderB);
        m_elevatorEncoder.setDistancePerPulse(kMetersPerPulse);
        absoluteEncoder = m_elevatorMotorM.getAbsoluteEncoder(Type.kDutyCycle);

        m_forwardLimit = m_elevatorMotorM.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        m_reverseLimit = m_elevatorMotorM.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);

        m_forwardLimit.enableLimitSwitch(false);
        m_reverseLimit.enableLimitSwitch(false);
        SmartDashboard.putBoolean("Forward Limit Enabled", m_forwardLimit.isLimitSwitchEnabled());
        SmartDashboard.putBoolean("Reverse Limit Enabled", m_reverseLimit.isLimitSwitchEnabled());

        m_elevatorMotorS.follow(m_elevatorMotorM);

        m_elevator = root.append(new MechanismLigament2d("elevator", kElevatorMinimumLength, 90));
        SmartDashboard.putData("Mech2d", mech);
    }

    public Command moveElevator(DoubleSupplier elevator) {
        return this.run(() -> {
            m_elevatorMotorM.set(elevator.getAsDouble());
        });
    }

    @Override
    public void periodic() {
        m_elevator.setLength(kElevatorMinimumLength + m_elevatorEncoder.getDistance());

        m_forwardLimit.enableLimitSwitch(SmartDashboard.getBoolean("Forward Limit Enabled", false));
        m_reverseLimit.enableLimitSwitch(SmartDashboard.getBoolean("Reverse Limit Enabled", false));

        SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
        SmartDashboard.putNumber("Absolute Encoder Position", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Absolute Encoder Velocity", absoluteEncoder.getVelocity());
    }
}
