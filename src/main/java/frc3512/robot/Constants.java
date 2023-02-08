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
            new Translation3d(0.0, Units.inchesToMeters(1.5), Units.inchesToMeters(39.0)),
            new Rotation3d(0.0, 0.0, 0.0));
  }

  /** Constants revolving around auton modes. */
  public static final class AutonConstants {
    public static final double maxVelocity = 1.0;
    public static final double maxAcceleration = 4.0;
  public static final class ElevatorConstants {
    public static final double kDt = 0.02;

    public static final int EncoderA = 0;
    public static final int EncoderB = 1;
    public static final int MotorAID = 20;
    public static final int MotorBID = 21;

    public static final double distancePerpulse = 0.01;
  }

  public static final class IntakeConstants {
    public static final int MotorID = 0;
  }

  /** Constants revolving around the swerve subsystem */
  public static final class SwerveConstants {
    /* Gyro Constants */
    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    public static final PathConstraints constraints =
        new PathConstraints(maxVelocity, maxAcceleration);

    public static final double xyControllerP = 1.0;
    public static final double thetaControllerP = 1.0;
  }
}
