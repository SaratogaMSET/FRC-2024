
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;


public class VisionSubsystem extends SubsystemBase {

    public VisionIO io;
    public VisionIOInputs inputs;

   
    public VisionSubsystem(VisionIO visionIO){

        io = visionIO;
        inputs = new VisionIOInputs();

    }

    // @Override
    // public Matrix<N3, N1> getScaledSTDDevs(){
    //     if(photonPoseEstimator.update().isPresent()){
    //         var estimation = photonPoseEstimator.update().get();
    //         double sumDistance = 0;
    //         for(var target: estimation.targetsUsed){
    //             var bestCamera = target.getBestCameraToTarget();
    //             sumDistance += Math.sqrt(Math.pow(bestCamera.getX(), 2) + Math.pow(bestCamera.getY(), 2) + Math.pow(bestCamera.getZ(), 2));
    //         }
    //         double targetsUsed = (double) estimation.targetsUsed.size();
    //         double averageDistance = sumDistance/ targetsUsed;
    //         return VecBuilder.fill((Vision.xyCoeff * averageDistance)/targetsUsed, (Vision.xyCoeff * averageDistance/targetsUsed), (Vision.rotationCoeff * averageDistance/targetsUsed));
    //     }
    //     else{
    //         return VecBuilder.fill(Vision.xyCoeff,Vision.xyCoeff,Vision.rotationCoeff);
    //     }
    // }

    @Override
    public void periodic() {
        io.updateInputs(inputs, new Pose3d());
    }

}