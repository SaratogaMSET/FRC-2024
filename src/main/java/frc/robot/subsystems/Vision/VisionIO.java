// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

/** Vision interface, holds data for logging and methods to interact with hardware. */
public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public double timestamp = 0.0;
    public double latency = 0.0;
    public List<PhotonTrackedTarget> targets =
        new ArrayList<>(); // TODO make protobuf work whenever that happens
    public double numTags = 0;
    public Pose3d pose = new Pose3d();
    public Optional<EstimatedRobotPose> estPose = Optional.empty();

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamp", timestamp);
      table.put("Latency", latency);
      for (int i = 0; i < targets.size(); i++) {
        VisionHelper.logPhotonTrackedTarget(targets.get(i), table, String.valueOf(i));
        numTags += 1;
      }
      table.put("NumTags", numTags);
      table.put("Pose", pose);
    }

    @Override
    public void fromLog(LogTable table) {
        timestamp = table.get("Timestamp", timestamp);
        latency = table.get("Latency", latency);
        for (int i = 0; i < table.get("number of tags", targets.size()); i++) {
          this.targets.add(VisionHelper.getLoggedPhotonTrackedTarget(table, String.valueOf(i)));
        }
        numTags = table.get("NumTags", numTags);
        pose = table.get("Pose", pose);
    }
  }
  
  /** Modifies the inputs object that is passed in, so the subsystems can pull updated data from it. robotPose is for feeding data into simulation */
  public default void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {}

  /** Enabled or disabled vision LEDs. */
  public default void setLeds(boolean enabled) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}