// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

/** 8033 + Docs */
public class VisionIOSim extends SubsystemBase implements VisionIO {
    VisionSystemSim sim =
      new VisionSystemSim("camera"); 
      //, 70, Constants.Vision.robotToCam, 9000, 1280, 800, 10);
    PhotonCamera camera = new PhotonCamera("camera");
    SimCameraProperties cameraProp = new SimCameraProperties();

    public PhotonPoseEstimator photonEstimator = null;
    public double lastEstTimestamp = 0;

  public VisionIOSim() {
    try {
      var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      photonEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.Vision.robotToCam);
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    sim.update(robotPose);
    var result = camera.getLatestResult();

    inputs.timestamp = result.getTimestampSeconds();
    inputs.targets = result.targets;
  }

    /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      var visionEst = photonEstimator.update();
      double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
      boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
      if (Robot.isSimulation()) {
          visionEst.ifPresentOrElse(
                  est ->
                          getSimDebugField()
                                  .getObject("VisionEstimation")
                                  .setPose(est.estimatedPose.toPose2d()),
                  () -> {
                      if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                  });
      }
      if (newResult) lastEstTimestamp = latestTimestamp;
      return visionEst;
  }

  @Override
  public double getTimestamp() {
    return lastEstTimestamp;
  }

  @Override
  public Optional<Pose2d> getPose2d(){
    if (getEstimatedGlobalPose().isPresent()) return Optional.of(getEstimatedGlobalPose().get().estimatedPose.toPose2d());
    else return Optional.ofNullable(null);
  }

  /** A Field2d for visualizing our robot and objects on the field. */
  public Field2d getSimDebugField() {
      if (!Robot.isSimulation()) return null;
      return sim.getDebugField();
  }
}