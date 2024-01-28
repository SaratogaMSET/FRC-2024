package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

public class Vision {
    public record VisionConstants(
        String cameraName, Transform3d robotToCam, PhotonCamera photonCamera) {}

    public record VisionConstantsSim(
        String cameraName, Transform3d robotToCam, PhotonCamera photonCamera, SimCameraProperties simProperties) {}
    

    private final VisionIO io;
    public final VisionIOInputs inputs = new VisionIOInputs();

    public Vision(final VisionIO io) {
        this.io = io;
    }

    /**
     * Update inputs without running the rest of the periodic logic. This is useful since these
     * updates need to be properly thread-locked.
     */
    public void updateInputs(Pose3d robotPose) {
        io.updateInputs(inputs, robotPose);
    }

}
