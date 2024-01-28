// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Vision.Vision.VisionConstantsSim;

/** 8033 + Docs + 6238 */
public class VisionIOSim implements VisionIO {

  public static VisionSystemSim sim = new VisionSystemSim("camera"); 
  PhotonPoseEstimator photonPoseEstimator;

  public double timestamp = 0;
  public PhotonPipelineResult result; 

  public VisionIOSim(VisionConstantsSim m_visionConstantsSim) {
    try {
      var field = FieldConstants.aprilTags;
      sim.addAprilTags(field);

      photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_visionConstantsSim.photonCamera(), m_visionConstantsSim.robotToCam());
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      PhotonCameraSim cameraSim = new PhotonCameraSim(m_visionConstantsSim.photonCamera(), m_visionConstantsSim.simProperties());
      // Enable the raw and processed streams. These are enabled by default.
      cameraSim.enableRawStream(true);
      cameraSim.enableProcessedStream(true);
      sim.addCamera(cameraSim, Constants.Vision.robotToCam);
      result = m_visionConstantsSim.photonCamera().getLatestResult();

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    /** Modifies the inputs object, while recieving pose data. */
    sim.update(robotPose);

    inputs.latency = result.getLatencyMillis() / 1000;
    inputs.timestamp = result.getTimestampSeconds();
    inputs.targets = result.getTargets();
    inputs.numTags = inputs.targets.size();
    inputs.estPose = photonPoseEstimator.update();

    // inputs.pose = robotPose; //TODO, do we want this? 
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return sim.getDebugField();
  }
}