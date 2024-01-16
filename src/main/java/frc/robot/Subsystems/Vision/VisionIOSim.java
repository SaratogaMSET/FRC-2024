// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import java.io.IOException;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** Add your docs here. */
public class VisionIOSim implements VisionIO {
    VisionSystemSim sim =
      new VisionSystemSim("camera"); 
      //, 70, Constants.Vision.robotToCam, 9000, 1280, 800, 10);
    PhotonCamera camera = new PhotonCamera("camera");
    SimCameraProperties cameraProp = new SimCameraProperties();

    // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
    // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
    Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
    // and pitched 15 degrees up.
    Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

  public VisionIOSim() {
    try {
      var field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
      field.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      sim.addAprilTags(field);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {
    sim.update(robotPose);
    var result = camera.getLatestResult();

    inputs.timestamp = result.getTimestampSeconds();
    inputs.timeSinceLastTimestamp = 0.0;
    inputs.targets = result.targets;
  }
}