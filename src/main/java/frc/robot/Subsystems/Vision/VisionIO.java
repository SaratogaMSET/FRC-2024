// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

/** Add your docs here. */
public interface VisionIO extends Subsystem {
  public static class VisionIOInputs {
    public double timestamp = 0.0;
    public double latency = 0.0;
    public List<PhotonTrackedTarget> targets =
        new ArrayList<>(); // TODO make protobuf work whenever that happens
    public double numTags = 0;
    public Pose3d pose = new Pose3d();
  }

  public default void updateInputs(VisionIOInputs inputs, Pose3d pose) {} ;

  public default Optional<Pose2d> getPose2d() {return Optional.ofNullable(null); } ;

  public default Matrix<N3, N1> getScaledSTDDevs() {return Constants.Vision.stateSTD; } ;

  public default double getTimestamp() {return 0.0; }; 
}