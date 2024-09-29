package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.Vision;
import frc.robot.FieldConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionIOReal implements VisionIO {
  PhotonCamera camera;
  Transform3d robotToCamera;

  PhotonPipelineResult result;
  PhotonPoseEstimator photonPoseEstimator;

  /** Creates a IO object to represent a physical hardware camera. */
  public VisionIOReal(int index) {
    // Allows for easy organization of multiple cameras.
    switch (index) {
      case 0:
        camera = new PhotonCamera("Arducam_9281_14_BL");
        robotToCamera = Vision.jawsCamera0;
        break;
      case 1:
        camera = new PhotonCamera("Arducam_9782_14_BR");
        robotToCamera = Vision.jawsCamera1;
        break;
      default:
        throw new RuntimeException("Invalid Index");
    }
    var field = FieldConstants.aprilTags;

    photonPoseEstimator =
        new PhotonPoseEstimator(
            field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
    photonPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    result = camera.getLatestResult();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    /** Modifies the inputs object, while recieving pose data. */
    result = camera.getLatestResult();

    // inputs.pipelineResult = result;

    inputs.latency = result.getLatencyMillis() / 1000;
    inputs.timestamp = result.getTimestampSeconds();
    inputs.averageAmbiguity =
        result.getTargets().stream().mapToDouble((target) -> target.getPoseAmbiguity()).sum()
            / result.getTargets().size();
    inputs.targetCount = result.getTargets().size();
    inputs.estPose = photonPoseEstimator.update();

    // inputs.pose = robotPose; //TODO, do we want this?
  }
}
