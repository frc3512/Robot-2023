package frc3512.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleArrayEntry;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.util.GlobalMeasurement;
import frc3512.robot.Constants;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
  public PhotonCamera photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);
  public PhotonPoseEstimator photonPoseEstimator;
  public AprilTagFieldLayout atfl;

  private ArrayList<GlobalMeasurement> measurements = new ArrayList<>();
  private PhotonTrackedTarget bestTarget;
  private GlobalMeasurement bestMeasurement;
  private boolean hasTarget = false;

  private final SpartanDoubleArrayEntry tagPoseEntry =
      new SpartanDoubleArrayEntry("/Diagnostics/Vision/Tag Pose");
  private final SpartanDoubleEntry timestampEntry =
      new SpartanDoubleEntry("/Diagnostics/Vision/Timestamp");
  private final SpartanDoubleEntry ambiguityEntry =
      new SpartanDoubleEntry("/Diagnostics/Vision/Tag Ambiguity");

  public Vision() {
    PhotonCamera.setVersionCheckEnabled(false);

    try {
      atfl = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      photonPoseEstimator =
          new PhotonPoseEstimator(
              atfl, PoseStrategy.MULTI_TAG_PNP, photonCamera, Constants.VisionConstants.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
  }

  public List<GlobalMeasurement> getMeasurements() {
    return measurements;
  }

  public PhotonTrackedTarget getBestTarget() {
    return bestTarget;
  }

  public GlobalMeasurement getBestMeasurement() {
    return bestMeasurement;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  /**
   * @param estimatedRobotPose The current best guess at robot pose
   * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
   *     of the observation. Assumes a planar field and the robot is always firmly on the ground
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (photonPoseEstimator == null) {
      return Optional.empty();
    }
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = photonCamera.getLatestResult();

    if (!result.hasTargets()) {
      hasTarget = false;
      measurements.clear();
      bestTarget = null;
      bestMeasurement = null;
      return;
    }

    hasTarget = true;

    bestTarget = result.getBestTarget();

    Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();

    Optional<Pose3d> feducialPos = atfl.getTagPose(bestTarget.getFiducialId());

    Pose3d bestPose;

    if (!feducialPos.isEmpty()) {
      bestPose =
          PhotonUtils.estimateFieldToRobotAprilTag(
              cameraToTarget, feducialPos.get(), Constants.VisionConstants.robotToCam);
    } else {
      bestPose = null;
    }
    if (bestPose == null) {
      bestMeasurement = null;
    } else {
      bestMeasurement =
          new GlobalMeasurement(
              new Pose2d(
                  bestPose.getX(), bestPose.getY(), new Rotation2d(bestPose.getRotation().getZ())),
              result.getTimestampSeconds(),
              bestTarget.getPoseAmbiguity());
    }

    tagPoseEntry.set(
        new double[] {
          bestMeasurement.pose.getX(),
          bestMeasurement.pose.getY(),
          bestMeasurement.pose.getRotation().getDegrees()
        });
    timestampEntry.set(bestMeasurement.timestampSeconds);
    ambiguityEntry.set(bestMeasurement.ambiguity);
  }
}
