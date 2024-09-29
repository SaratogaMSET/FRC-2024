// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

/** Vision interface, holds data for logging and methods to interact with hardware. */
public interface VisionIO {
  public static class VisionIOInputs implements LoggableInputs {
    public double timestamp = 0.0;
    public double latency = 0.0;
    public PhotonPipelineResult pipelineResult = new PhotonPipelineResult();
    public double averageAmbiguity = 0.0;
    public double targetCount = 0.0; // Number of targets seen
    // public List<PhotonTrackedTarget> targets =
    // new ArrayList<>();
    // public double numTags = 0;
    public Pose3d pose = new Pose3d();
    public Optional<EstimatedRobotPose> estPose = Optional.empty();

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamp", timestamp);
      table.put("Latency", latency);
      // table.put("Pipeline Result", pipelineResult);
      table.put("Average Ambiguity", averageAmbiguity);
      table.put("Number of Targets Tracked", targetCount);
      table.put("Pose", pose);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamp = table.get("Timestamp", timestamp);
      latency = table.get("Latency", latency);
      // pipelineResult = table.get("Pipeline Result", pipelineResult);
      averageAmbiguity = table.get("Average Ambiguity", averageAmbiguity);
      targetCount = table.get("Number of Targets Tracked", targetCount);
      pose = table.get("Pose", pose);
    }
  }

  /**
   * Modifies the inputs object that is passed in, so the subsystems can pull updated data from it.
   * robotPose is for feeding data into simulation
   */
  public default void updateInputs(VisionIOInputs inputs, Pose3d robotPose) {}

  /** Enabled or disabled vision LEDs. */
  public default void setLeds(boolean enabled) {}

  /** Sets the pipeline number. */
  public default void setPipeline(int pipeline) {}
}
