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

  /* Field related constants */
  public static final class FieldConstants {
    // List of possible scoring locations as Pose2d objects
    public static final List<Pose2d> scoringPositions =
        List.of(
            new Pose2d(
                new Translation2d(0.555, 7.436),
                Rotation2d.fromRadians(Math.PI)), // Red loading double station
            new Pose2d(new Translation2d(0.555, 6.146), Rotation2d.fromRadians(Math.PI)),
            new Pose2d(
                new Translation2d(15.03, 5.061),
                Rotation2d.fromDegrees(0.0)), // Red node scoring locations
            new Pose2d(new Translation2d(15.03, 4.405), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 3.846), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 3.298), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 2.74), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 2.2), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 1.62), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 1.06), Rotation2d.fromDegrees(0.0)),
            new Pose2d(new Translation2d(15.03, 0.52), Rotation2d.fromDegrees(0.0)),
            new Pose2d(
                new Translation2d(15.64, 7.430),
                Rotation2d.fromDegrees(0.0)), // Blue loading double substation
            new Pose2d(new Translation2d(15.64, 6.16), Rotation2d.fromDegrees(0.0)),
            new Pose2d(
                new Translation2d(1.598, 4.996),
                Rotation2d.fromRadians(-Math.PI)), // Blue node scoring locations
            new Pose2d(new Translation2d(1.598, 4.373), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 3.85), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 3.3), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 2.75), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 2.2), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 1.63), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 1.05), Rotation2d.fromRadians(-Math.PI)),
            new Pose2d(new Translation2d(1.598, 0.5), Rotation2d.fromRadians(-Math.PI)));
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
    public static final String cameraName = "USB_GS_Camera";

    // Robot to camera transform
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.0, Units.inchesToMeters(1.5), Units.inchesToMeters(39.0)),
            new Rotation3d(0.0, 0.0, 0.0));

    public static double yawOffsetDegrees = 3.5;

    public static double turnP = 0.2;
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
    public static final double stowValue = 3.80; // 2.70

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
