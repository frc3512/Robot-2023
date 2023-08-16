package frc3512.robot;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/** Constants for the robot project */
public final class Constants {

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

  /** Constants revolving around scoring */
  public static class ScoringConstants {
    public static final double elevatorLeveled = 0.0;
    public static final double elevatorCubeL3 = 0.26;
    public static final double elevatorConeL2 = 0.18;
    public static final double elevatorConeL3 = 0.35;

    public static final double armIntake = 1.33;
    public static final double armCubeL2 = 2.25;
    public static final double armCubeL3 = 1.75;
    public static final double armConeL2 = 1.75;
    public static final double armConeL3 = 1.43;
    public static final double armConeHP = 2.30;
    public static final double armCubeHP = 2.42;

    public static final List<Pose2d> redScoringPositions =
        List.of(
            new Pose2d(new Translation2d(14.70, 5.00), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 4.42), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 3.83), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 3.30), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 2.76), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 2.19), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 1.69), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 1.09), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(14.70, 0.51), Rotation2d.fromDegrees(0.0)));
    // public static final List<Pose2d> blueScoringPositions;
  }

  /** Constants revolving around the vision subsystem. */
  public static final class VisionConstants {
    // Camera name
    public static final String cameraName = "OV5647";

    // Robot to camera transform
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.0, Units.inchesToMeters(1.5), Units.inchesToMeters(39.0)),
            new Rotation3d(0.0, 0.0, 0.0));
  }

  /** Constants revolving around the LED subsystem. */
  public static class LEDConstants {
    public static final int pwmPort = 0;
    public static final int ledBufferLength = 60;
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
    public static final double maxAccelerationMeterPerSecondSquared = 1.0;

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
    public static final double stowValue = 2.7;

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
