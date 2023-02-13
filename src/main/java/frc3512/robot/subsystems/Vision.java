package frc3512.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc3512.lib.logging.SpartanDoubleEntry;
import frc3512.lib.logging.SpartanPose2dEntry;
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
  public PhotonCamera photonCamera;
  public PhotonPoseEstimator photonPoseEstimator;
  public AprilTagFieldLayout atfl;

  private ArrayList<GlobalMeasurement> measurements = new ArrayList<>();
  private PhotonTrackedTarget bestTarget;
  private GlobalMeasurement bestMeasurement;
  private boolean hasTarget = false;

  private SpartanPose2dEntry tagPoseEntry;
  private SpartanDoubleEntry timestampEntry;
  private SpartanDoubleEntry ambiguityEntry;

  public Vision() {
    photonCamera = new PhotonCamera(Constants.VisionConstants.cameraName);

    try {
      atfl = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    } catch (IOException e) {
      e.printStackTrace();
    }

    // Create pose estimator
    photonPoseEstimator =
        new PhotonPoseEstimator(
            atfl,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            photonCamera,
            Constants.VisionConstants.robotToCam);

    tagPoseEntry = new SpartanPose2dEntry("/Diagnostics/Vision/Tag Pose");
    timestampEntry = new SpartanDoubleEntry("/Diagnostics/Vision/Timestamp");
    ambiguityEntry = new SpartanDoubleEntry("/Diagnostics/Vision/Tag Ambiguity");
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

    tagPoseEntry.set(bestMeasurement.pose);
    timestampEntry.set(bestMeasurement.timestampSeconds);
    ambiguityEntry.set(bestMeasurement.ambiguity);
  }
}
