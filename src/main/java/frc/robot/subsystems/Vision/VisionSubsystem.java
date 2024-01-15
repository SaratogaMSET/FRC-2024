
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;


public class VisionSubsystem extends SubsystemBase {

    PhotonCamera ll2 = new PhotonCamera("OV5647");
    public PhotonPoseEstimator photonPoseEstimator;

   
    AprilTagFieldLayout field = null;  //TODO: fill in 

    private double[] distances = new double[]{
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1,
        -1
    };

    private double[] getDistances() {
        distances = new double[]{
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1,
            -1
        };

        for (var r : getTargets()) {
            var tmp = r.getBestCameraToTarget();
            distances[(int) r.getFiducialId() - 1] = Math.hypot(tmp.getX(), tmp.getZ()); 
            // we only care about the horizontal distance not the vertical. 
        }

        return distances;
    }
    
    public VisionSubsystem(){
        try {
            field = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile); //why does this not work man
        } catch (Exception e){
            SmartDashboard.putNumber("ITS JOEVER", 1);
        }

        //field = FieldConstants.aprilTags;
        photonPoseEstimator = new PhotonPoseEstimator(field, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ll2, Constants.Vision.robotToCam);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
    }
    // Construct PhotonPoseEstimator
    public PhotonPipelineResult ll2_Result(){
        return ll2.getLatestResult();
    }

    public PhotonPipelineResult getLatestResults(){
        if (ll2_Result().hasTargets()){
            return ll2_Result();
        }
        return ll2_Result();
    }

    public boolean has_targets(){
        return ll2_Result().hasTargets();
    }

    public List<PhotonTrackedTarget> getTargets(){
        if (has_targets()) return ll2_Result().getTargets();
        return new ArrayList<PhotonTrackedTarget>();
    }

    /**
     * @return the best target acquired by the camera, can return Null
     */
    public Optional<PhotonTrackedTarget> getBestTarget(){
        if (has_targets()) return Optional.of(getLatestResults().getBestTarget());
        return Optional.ofNullable(null);
        //returns null if there is no target*
        //TODO: change other dependent methods to use Optional interface instead of try catch goofyness. 
    }

    public int getTagID(){
        try {
            return getBestTarget().get().getFiducialId();
        } catch (Exception e) {
            System.out.println("no targets");
            return -1;
        }
    }

    public Transform3d getCamTran(){
        try {
            return getBestTarget().get().getBestCameraToTarget();
        } catch (Exception e) {
            System.out.println("No Targets?");
            return new Transform3d();
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return photonPoseEstimator.update();
    }

    public Optional<Pose2d> getPose2d(){
        if (getEstimatedGlobalPose().isPresent()) return Optional.of(getEstimatedGlobalPose().get().estimatedPose.toPose2d());
        else return Optional.ofNullable(null);
    }

    public double getLatency(){
        return ll2_Result().getLatencyMillis();
    }

    public Vector<N3> getScaledSTDDevs(){
        if(photonPoseEstimator.update().isPresent()){
            var estimation = photonPoseEstimator.update().get();
            double sumDistance = 0;
            for(var target: estimation.targetsUsed){
                var bestCamera = target.getBestCameraToTarget();
                sumDistance += Math.sqrt(Math.pow(bestCamera.getX(), 2) + Math.pow(bestCamera.getY(), 2) + Math.pow(bestCamera.getZ(), 2));
            }
            double targetsUsed = (double) estimation.targetsUsed.size();
            double averageDistance = sumDistance/ targetsUsed;
            return VecBuilder.fill((Vision.xyCoeff * averageDistance)/targetsUsed, (Vision.xyCoeff * averageDistance/targetsUsed), (Vision.rotationCoeff * averageDistance/targetsUsed));
        }
        else{
            return VecBuilder.fill(Vision.xyCoeff,Vision.xyCoeff,Vision.rotationCoeff);
        }
    }

    public double getTimestamp(){
        return Timer.getFPGATimestamp() - getLatency();
    }

    public int getPipelineIndex(){
        return ll2.getPipelineIndex();
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pipeline INdex", getPipelineIndex());
  }


}