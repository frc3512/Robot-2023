package frc3512.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc3512.lib.util.SwerveModuleConstants;

/** Constants for the robot project */
public final class Constants {

  /** General robot constants */
  public static final class GeneralConstants {
    // Enable or disable competition mode
    public static final boolean tuningMode = true;

    // Joystick axis deadband for the swerve drive
    public static final double swerveDeadband = 0.1;

    // Value to voltage compensate the motors for on the swerve drive
    public static final double voltageCompSwerve = 12.0;
  }

  /** Constants revolving around joysticks */
  public static class OperatorConstants {
    // 1st Xbox controller port
    public static final int xboxController1Port = 0;

    // 2nd Xbox controller port
    public static final int xboxController2Port = 1;
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

  /** Constants revolving around the swerve subsystem */
  public static final class SwerveConstants {
    /* Gyro Constants */
    public static final int pigeonID = 6;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.73);
    public static final double wheelBase = Units.inchesToMeters(21.73);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 50;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 2; // Proto: 15
      public static final int angleMotorID = 1; // Proto: 16
      public static final int canCoderID = 4;
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(25.576171875 + 180.0); // Proto: -112.5
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 16; // Proto: 2
      public static final int angleMotorID = 15; // Proto: 1
      public static final int canCoderID = 1; // Proto: 3
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(142.822265625 + 180.0); // Proto: -125.59569549560548
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4; // Proto: 14
      public static final int angleMotorID = 3; // Proto: 13
      public static final int canCoderID = 3; // Proto : 2
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(163.125 + 180.0); // Proto: 106.96289825439453
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 13; // Proto: 4
      public static final int angleMotorID = 14; // Proto: 3
      public static final int canCoderID = 2; // Proto: 1
      public static final Rotation2d angleOffset =
          Rotation2d.fromDegrees(-58.798828125 + 180.0); // Proto: 149.677734375
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
}
