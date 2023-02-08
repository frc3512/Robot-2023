package frc3512.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

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

    private final CANSparkMax m_elevatorMotor;
    private final Encoder m_elevatorEncoder;

    private MechanismLigament2d m_elevator;

    Mechanism2d mech = new Mechanism2d(3,3);
    MechanismRoot2d root = mech.getRoot("climber", 2, 0);

    public Elevator() {
        m_elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.MotorAID, CANSparkMaxLowLevel.MotorType.kBrushless);
        m_elevatorEncoder = new Encoder(ElevatorConstants.EncoderA, ElevatorConstants.EncoderB);
        m_elevatorEncoder.setDistancePerPulse(kMetersPerPulse);

        m_elevator = root.append(new MechanismLigament2d("elevator", kElevatorMinimumLength, 90));
        SmartDashboard.putData("Mech2d", mech);
    }

    public Command moveElevator(DoubleSupplier elevator) {
        return this.run(() -> {
            m_elevatorMotor.set(elevator.getAsDouble());
        });
    }

    @Override
    public void periodic() {
        m_elevator.setLength(kElevatorMinimumLength + m_elevatorEncoder.getDistance());
    }
}
