package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;

public class VisionIOReal implements VisionIO {
    PhotonCamera camera = new PhotonCamera("OV5647");

    PhotonPipelineResult result;
    PhotonPoseEstimator photonPoseEstimator;

    public VisionIOReal() {
        var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.Vision.robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        result = camera.getLatestResult();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
        /** Modifies the inputs object, while recieving pose data. */

        inputs.latency = result.getLatencyMillis() / 1000;
        inputs.timestamp = result.getTimestampSeconds();
        inputs.targets = result.getTargets();
        inputs.numTags = inputs.targets.size();
        inputs.estPose = photonPoseEstimator.update();

        inputs.pose = robotPose; //TODO, do we want this? 
    }
}
