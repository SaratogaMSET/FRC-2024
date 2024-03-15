// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FieldConstants;
import frc.robot.Robot;

/** 8033 + Docs + 6238 */
public class VisionIOSim implements VisionIO {

  public static VisionSystemSim sim = new VisionSystemSim("camera"); 
  public PhotonPoseEstimator photonPoseEstimator;
  public PhotonPipelineResult result;
  public PhotonCamera camera;
  public PhotonCameraSim cameraSim;
  public SimCameraProperties cameraProperties;
  public Transform3d robotToCam;

  public double timestamp = 0;

  public VisionIOSim(int index) {
    try {
      var field = FieldConstants.aprilTags;
      sim.addAprilTags(field);

      switch (index) {
        case 0:
          camera = new PhotonCamera("SimCam1");
          cameraProperties = SimCameraProperties.LL2_640_480();
          cameraSim = new PhotonCameraSim(camera, cameraProperties);
          robotToCam = new Transform3d(-Units.inchesToMeters(12), Units.inchesToMeters(11), Units.inchesToMeters(6.5), new Rotation3d(Math.toRadians(15), 0, Math.toRadians(215)));
          break;
        case 1:
          
          camera = new PhotonCamera("SimCam2");
          cameraProperties = SimCameraProperties.LL2_640_480();
          cameraSim = new PhotonCameraSim(camera, cameraProperties);
          robotToCam = new Transform3d(-Units.inchesToMeters(12), -Units.inchesToMeters(11), Units.inchesToMeters(6.5), new Rotation3d(Math.toRadians(15), 0, Math.toRadians(125)));
          break;
        default:
          throw new RuntimeException("Invalid Camera Index");
      }

      photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);
      photonPoseEstimator.setTagModel(TargetModel.kAprilTag36h11);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      // Enable the raw and processed streams. These are enabled by default.
      cameraSim.enableRawStream(true);
      cameraSim.enableProcessedStream(true);
      sim.addCamera(cameraSim, robotToCam);
      // result = camera.getLatestResult();

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    /** Modifies the inputs object, while recieving pose data. */
    sim.update(robotPose);
    result = camera.getLatestResult();

    if (result == null) return;
    inputs.pipelineResult = result;
    inputs.latency = result.getLatencyMillis() / 1000;
    inputs.timestamp = result.getTimestampSeconds();
    inputs.averageAmbiguity = result.getTargets().stream().mapToDouble((target) -> target.getPoseAmbiguity()).sum() / result.getTargets().size();
    inputs.targetCount = result.getTargets().size();
    inputs.estPose = photonPoseEstimator.update();

    // inputs.pose = robotPose; //TODO, do we want this? 
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return sim.getDebugField();
  }
}