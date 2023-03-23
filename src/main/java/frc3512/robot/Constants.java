package frc3512.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/** Constants for the robot project */
public final class Constants {

  /* Field related constants */
  public static final class FieldConstants {
    // List of possible scoring locations as Pose2d objects
    public static final List<Pose2d> scoringPositions =
        List.of(
            // Red node scoring locations
            new Pose2d(1.92, 5.00, Rotation2d.fromRadians(-Math.PI)), 
            new Pose2d(1.92, 4.42, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(1.92, 3.83, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(1.92, 3.30, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(1.92, 2.76, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(1.92, 2.19, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(1.92, 1.69, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(1.92, 1.09, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(1.92, 0.51, Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(14.70, 5.00, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 4.42, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 3.83, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 3.30, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 2.76, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 2.19, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 1.69, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 1.09, Rotation2d.fromDegrees(0.0)),
            new Pose2d(14.70, 0.51, Rotation2d.fromDegrees(0.0)));
  }

  /** General robot constants */
  public static final class GeneralConstants {
    // Enable or disable competition mode
    public static final boolean tuningMode = true;

    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;

    public static final double voltageComp = 10.0;

    // Hold time on motor brakes when disabled
    public static final double wheelLockTime = 10;

    public static final double robotMass = (148 - 20.3) * 0.453592;
    public static final double chassisMass = robotMass;
    public static final Translation3d chassisCG = new Translation3d(0, 0, Units.inchesToMeters(8));
    public static final double loopTime = 0.13;
  }

  /** Constants revolving around joysticks */
  public static class OperatorConstants {
    // Driver controller port
    public static final int driverControllerPort = 0;

    // Appendage controller port
    public static final int appendageControllerPort = 1;
  }

  /** Constants revolving around the vision subsystem. */
  public static final class VisionConstants {
    // Camera name
    public static final String cameraName = "OV5647";

    // Robot to camera transform
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.0, Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
            new Rotation3d(0.0, Units.degreesToRadians(0.0), 0.0));
  }

  /** Constants revolving around the elevator subsystem. */
  public static final class ElevatorConstants {
    public static final int leftMotorID = 17;
    public static final int rightMotorID = 18;
    public static final int encoderA = 0;
    public static final int encoderB = 1;

    public static final int currentLimit = 70;
    public static final double teleopSpeedMultiplier = 0.4;

    public static final double pGain = 85.0;
    public static final double iGain = 0.0;
    public static final double dGain = 0.0;

    public static final int averageSampleSize = 10;
    public static final double distancePerPulse =
        (Math.PI * 2.0 * Units.inchesToMeters(1.751)) / 8192;

    public static final double maxVelocityMeterPerSecond = 3.0;
    public static final double maxAccelerationMeterPerSecondSquared = 0.75;

    public static final double minHeight = 0.0;
    public static final double maxHeight = 0.35;
  }

  /** Constants revolving around the arm subsystem. */
  public static final class ArmConstants {
    public static final int leftMotorID = 19;
    public static final int rightMotorID = 20;

    public static final int currentLimit = 40;
    public static final double teleopSpeed = 0.5;

    public static final double pGain = 10.7;
    public static final double iGain = 0.0;
    public static final double dGain = 0.0;

    public static final double positionConversionFactor = (Math.PI * 2.0);
    public static final double armOffset = 0.0;
    public static final double stowValue = 2.70;

    public static final double maxVelocityRadPerSecond = 12.0;
    public static final double maxAccelerationRadPerSecSquared = 9.0;

    public static final double minAngle = 0.05;
    public static final double maxAngle = 6.0;
  }

  /** Constants revolving around the intake subsystem. */
  public static final class IntakeConstants {
    public static final int intakeMotorID = 21;

    public static final int currentLimit = 40;

    public static final double motorSpeed = 0.9;
  }

  /** Constants revolving around auton modes. */
  public static final class AutonConstants {

    public static final double maxVelocity = 3.0;
    public static final double maxAcceleration = 3.0;

    public static final PathConstraints constraints =
        new PathConstraints(AutonConstants.maxVelocity, AutonConstants.maxAcceleration);

    public static final double xyControllerP = 3.0;
    public static final double thetaControllerP = 2.5;
  }
}
