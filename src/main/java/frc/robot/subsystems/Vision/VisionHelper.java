// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LogTable;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionHelper {

    // 8033. yall are actual degens for rewrite half of photonvision in here.

    public static void logPhotonTrackedTarget(
      PhotonTrackedTarget target, LogTable table, String name) {

        logTransform3d(target.getBestCameraToTarget(), table, name);
        logTransform3d(target.getAlternateCameraToTarget(), table, "alt " + name);
        logCorners(target, table, name);

        table.put("yaw " + name, target.getYaw());
        table.put("pitch " + name, target.getPitch());
        table.put("area " + name, target.getArea());
        table.put("skew " + name, target.getSkew());
        table.put("fiducial id " + name, target.getFiducialId());
        table.put("pose ambiguity " + name, target.getPoseAmbiguity());
    }

  public static void logCorners(PhotonTrackedTarget target, LogTable table, String name) {
        double[] detectedCornersX = new double[4];
        double[] detectedCornersY = new double[4];
        double[] minAreaRectCornersX = new double[4];
        double[] minAreaRectCornersY = new double[4];

        for (int i = 0; i < 4; i++) {
        detectedCornersX[i] = target.getDetectedCorners().get(i).x;
        detectedCornersY[i] = target.getDetectedCorners().get(i).y;
        minAreaRectCornersX[i] = target.getMinAreaRectCorners().get(i).x;
        minAreaRectCornersY[i] = target.getMinAreaRectCorners().get(i).y;
        }
        table.put("detected corners x " + name, detectedCornersX);
        table.put("detected corners y " + name, detectedCornersY);
        table.put("min area rect corners x " + name, minAreaRectCornersX);
        table.put("min area rect corners Y " + name, minAreaRectCornersY);
    }

    public static void logTransform3d(Transform3d transform3d, LogTable table, String name) {
        double rotation[] = new double[4];
        rotation[0] = transform3d.getRotation().getQuaternion().getW();
        rotation[1] = transform3d.getRotation().getQuaternion().getX();
        rotation[2] = transform3d.getRotation().getQuaternion().getY();
        rotation[3] = transform3d.getRotation().getQuaternion().getZ();
        table.put("rotation " + name, rotation);

        double translation[] = new double[3];
        translation[0] = transform3d.getTranslation().getX();
        translation[1] = transform3d.getTranslation().getY();
        translation[2] = transform3d.getTranslation().getZ();
        table.put("translation " + name, translation);
    }

  public static Transform3d getLoggedTransform3d(double[] translation, double[] rotation) {
    Transform3d transform3d =
        new Transform3d(
            new Translation3d(translation[0], translation[1], translation[2]),
            new Rotation3d(new Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])));

    return transform3d;
    }

    public static Transform3d getLoggedTransform3d(LogTable table, String name) {
        double[] rotation = table.get("rotation " + name, new double[4]);
        double[] translation = table.get("translation " + name, new double[3]);
        return getLoggedTransform3d(translation, rotation);
    }

    public static PhotonTrackedTarget getLoggedPhotonTrackedTarget(LogTable table, String name) {
        double[] translation = table.get("translation " + name, new double[3]);
        double[] rotation = table.get("rotation " + name, new double[4]);
        double[] altTranslation = table.get("translation alt " + name, new double[3]);
        double[] altRotation = table.get("rotation alt " + name, new double[4]);
        double[] detectedCornersX = table.get("detected corners x " + name, new double[4]);
        double[] detectedCornersY = table.get("detected corners y " + name, new double[4]);
        double[] minAreaRectCornersX = table.get("min area rect corners x " + name, new double[4]);
        double[] minAreaRectCornersY = table.get("min area rect corners y " + name, new double[4]);

        List<TargetCorner> detectedCorners = new ArrayList<>();
        List<TargetCorner> minAreaRectCorners = new ArrayList<>();

        for (int i = 0; i < 4; i++) {
        detectedCorners.add(new TargetCorner(detectedCornersX[i], detectedCornersY[i]));
        minAreaRectCorners.add(new TargetCorner(minAreaRectCornersX[i], minAreaRectCornersY[i]));
        }
        Transform3d pose = getLoggedTransform3d(translation, rotation);
        Transform3d altPose = getLoggedTransform3d(altTranslation, altRotation);
        return (new PhotonTrackedTarget(
            table.get("yaw " + name, -1),
            table.get("pitch " + name, -1),
            table.get("area " + name, -1),
            table.get("skew " + name, -1),
            (int) (table.get("fiducial id " + name, -1)),
            pose,
            altPose,
            table.get("pose ambiguity " + name, -1),
            minAreaRectCorners,
            detectedCorners));
    } 

  // 5026
    public static record UnitDeviationParams(
        double distanceMultiplier, double eulerMultiplier, double minimum) {

        private double computeUnitDeviation(double averageDistance) {
            return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
        }
    }

  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    private Matrix<N3, N1> computeDeviation(double averageDistance) {
      return MatBuilder.fill(
          Nat.N3(),
          Nat.N1(),
          xParams.computeUnitDeviation(averageDistance),
          yParams.computeUnitDeviation(averageDistance),
          thetaParams.computeUnitDeviation(averageDistance));
    }

    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }
  }

    public static record VisionMeasurement(
      EstimatedRobotPose estimation, Matrix<N3, N1> confidence) {}

// Pretty sure this is useless? 
//   public static Matrix<N3, N1> findVisionMeasurements(EstimatedRobotPose estimation) {
//     double sumDistance = 0;
//     for (var target : estimation.targetsUsed) {
//       var t3d = target.getBestCameraToTarget();
//       sumDistance +=
//           Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
//     }
//     double avgDistance = sumDistance / estimation.targetsUsed.size();

//     var deviation =
//         TAG_COUNT_DEVIATION_PARAMS
//             .get(
//                 MathUtil.clamp(
//                     estimation.targetsUsed.size() - 1, 0, TAG_COUNT_DEVIATION_PARAMS.size() - 1))
//             .computeDeviation(avgDistance);

//     // System.out.println(
//     //     String.format(
//     //         "with %d tags at smallest distance %f and pose ambiguity factor %f, confidence
//     // multiplier %f",
//     //         estimation.targetsUsed.size(),
//     //         smallestDistance,
//     //         poseAmbiguityFactor,
//     //         confidenceMultiplier));

//     return deviation;
//   }
  // Reject unreasonable vision poses
  public static void sanityCheck(VisionMeasurement measurement, EstimatedRobotPose estimatedPose) {
    while (measurement != null) { // Could change depending on what the climb is
      if (Math.abs(measurement.estimation.estimatedPose.getZ()) > 0.5) {
        continue;
      }
      // Skip single-tag measurements with too-high ambiguity.
      if (measurement.estimation.targetsUsed.size() < 2
          && measurement
                  .estimation
                  .targetsUsed
                  .get(0)
                  .getBestCameraToTarget()
                  .getTranslation()
                  .getNorm()
              > Units.feetToMeters(13)) {
        continue;
      }
    }
  }
}