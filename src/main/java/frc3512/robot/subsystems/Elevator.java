package frc3512.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private static double kDt = ElevatorConstants.kDt;

    private final Encoder m_Encoder = new Encoder(ElevatorConstants.EncoderA, ElevatorConstants.EncoderB);
    private final MotorController m_motor = new CANSparkMax(ElevatorConstants.MotorAID, MotorType.kBrushless); 

    private final TrapezoidProfile.Constraints m_Constraints = 
        new TrapezoidProfile.Constraints(kDt, kDt);
    private final ProfiledPIDController m_controller =
        new ProfiledPIDController(kDt, kDt, kDt, m_Constraints);

    public void setup() {
        m_Encoder.setDistancePerPulse(ElevatorConstants.distancePerpulse);
    }

    public void controllerPeriodic() {
        m_motor.set(m_controller.calculate(m_Encoder.getDistance()));
    }

    public void setGoal(double goalNumber) {
        m_controller.setGoal(goalNumber);
    }

    @Override
    public void periodic() {
        controllerPeriodic(); 
    }
}