// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants;
import frc.robot.Robot;

/** 8033 + Docs + 6238 */
public class VisionIOSim implements VisionIO {

  VisionSystemSim sim = new VisionSystemSim("camera"); 
    //, 70, Constants.Vision.robotToCam, 9000, 1280, 800, 10);
  PhotonCamera camera = new PhotonCamera("camera");
  SimCameraProperties cameraProp = new SimCameraProperties();
  

  PhotonPoseEstimator photonPoseEstimator;

  public double timestamp = 0;
  public PhotonPipelineResult result;

  public VisionIOSim() {
    try {
      var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);

      photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.Vision.robotToCam);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      cameraProp.setCalibration(
                1280,
                720,
                MatBuilder.fill(
                        Nat.N3(),
                        Nat.N3(),
                        // intrinsic
                        1011.3749416937393,
                        0.0,
                        645.4955139388737,
                        0.0,
                        1008.5391755084075,
                        508.32877656020196,
                        0.0,
                        0.0,
                        1.0),
                VecBuilder.fill( // distort
                        0.13730101577061535,
                        -0.2904345656989261,
                        8.32475714507539E-4,
                        -3.694397782014239E-4,
                        0.09487962227027584));
        cameraProp.setCalibError(0.37, 0.06);
        cameraProp.setFPS(7);
        cameraProp.setAvgLatencyMs(60);
        cameraProp.setLatencyStdDevMs(20);
      PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
      // Enable the raw and processed streams. These are enabled by default.
      cameraSim.enableRawStream(true);
      cameraSim.enableProcessedStream(true);
      sim.addCamera(cameraSim, Constants.Vision.robotToCam);
      result = camera.getLatestResult();

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

    inputs.pose = robotPose; //TODO, do we want this? 
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return sim.getDebugField();
  }
}