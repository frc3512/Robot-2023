package frc3512.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.vision.PhotonPoseEstimator;
import frc3512.lib.vision.PhotonPoseEstimator.PoseStrategy;
import frc3512.robot.Constants;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPoseEstimator estimator;
  private AprilTagFieldLayout layout;

  private PhotonPipelineResult result;
  private PhotonTrackedTarget bestTarget;

  private boolean haveTargets = false;
  private double lastYaw = 0.0;
  private double lastDistance = 0.0;
  private Pose2d lastRobotPose = new Pose2d();
  private Pose2d globalPose = new Pose2d();
  private double globalTimestamp = 0.0;

  private SpartanDoubleEntry yawEntry;
  private SpartanDoubleEntry distanceEntry;

  public Vision() {
    camera = new PhotonCamera(Constants.VisionConstants.cameraName);
    PhotonCamera.setVersionCheckEnabled(false);

    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (DriverStation.getAlliance() == Alliance.Blue) {
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } else if (DriverStation.getAlliance() == Alliance.Red) {
      layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
    }

    estimator =
        new PhotonPoseEstimator(
            layout,
            PoseStrategy.CLOSEST_TO_LAST_POSE,
            camera,
            Constants.VisionConstants.robotToCam);

    yawEntry = new SpartanDoubleEntry("/Diagnostics/Vision/Yaw");
    distanceEntry = new SpartanDoubleEntry("/Diagnostics/Vision/Distance");
  }

  public void setRobotPose(Pose2d robotPose) {
    if (robotPose != null) {
      lastRobotPose = robotPose;
    } else {
      lastRobotPose = new Pose2d();
    }
  }

  public boolean hasTargets() {
    return haveTargets;
  }

  public double getYaw() {
    if (haveTargets) {
      return bestTarget.getYaw();
    } else {
      return lastYaw;
    }
  }

  public double getDistanceFromTarget() {
    if (haveTargets) {
      return PhotonUtils.getDistanceToPose(
          lastRobotPose, layout.getTagPose(bestTarget.getFiducialId()).get().toPose2d());
    } else {
      return lastDistance;
    }
  }

  public Pose2d estimateGlobalPose(Pose2d previousPose) {
    if (hasTargets()) {
      estimator.setLastPose(previousPose);
      var camPose = estimator.update();

      if (camPose.isPresent()) {
        var pose = camPose.get().estimatedPose;
        var timestamp = camPose.get().timestampSeconds;
        globalPose = pose.toPose2d();
        globalTimestamp = timestamp;
      }
    } else {
      globalPose = lastRobotPose;
      globalTimestamp = Timer.getFPGATimestamp() - result.getLatencyMillis();
    }
    return globalPose;
  }

  public double getGlobalTimestamp() {
    return globalTimestamp;
  }

  @Override
  public void periodic() {
    if (RobotBase.isReal()) {
      result = camera.getLatestResult();
      bestTarget = result.getBestTarget();
      haveTargets = result.hasTargets();
      if (haveTargets) {
        lastYaw = bestTarget.getYaw();
        lastDistance = getDistanceFromTarget();
      }
      yawEntry.set(lastYaw);
      distanceEntry.set(lastDistance);
    }
  }
}
