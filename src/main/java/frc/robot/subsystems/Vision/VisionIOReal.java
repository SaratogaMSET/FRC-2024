package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Vision.Vision.VisionConstants;

public class VisionIOReal implements VisionIO {
    PhotonCamera camera;

    PhotonPipelineResult result;
    PhotonPoseEstimator photonPoseEstimator;

    public VisionIOReal(VisionConstants visionConstants) {
        var field = FieldConstants.aprilTags;

        camera = visionConstants.photonCamera(); /** Autogen Getters */

        photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, visionConstants.robotToCam());
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

        // inputs.pose = robotPose; //TODO, do we want this? 
    }
}