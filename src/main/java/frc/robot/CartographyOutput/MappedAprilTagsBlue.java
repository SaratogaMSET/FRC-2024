package frc.robot.CartographyOutput;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.ArrayList;
import java.util.List;

public class MappedAprilTagsBlue {
  public AprilTagFieldLayout MappedLayout() {
    return new AprilTagFieldLayout(this.aprilTags, 16.541, 8.211);
  }

  List<AprilTag> aprilTags = new ArrayList<AprilTag>();

  public MappedAprilTagsBlue() {
    aprilTags.add(
        new AprilTag(
            1,
            new Pose3d(
                new Translation3d(15.080439, 0.251021, 1.355803),
                new Rotation3d(new Quaternion(0.500023, 0.000003, -0.000002, 0.866012)))));
    aprilTags.add(
        new AprilTag(
            2,
            new Pose3d(
                new Translation3d(16.185157, 0.883466, 1.355817),
                new Rotation3d(new Quaternion(0.500021, 0.000002, -0.000001, 0.866013)))));
    aprilTags.add(
        new AprilTag(
            3,
            new Pose3d(
                new Translation3d(16.579948, 4.983100, 1.451077),
                new Rotation3d(new Quaternion(-0.000009, 0.000002, 0.000001, 1.000000)))));
    aprilTags.add(
        new AprilTag(
            4,
            new Pose3d(
                new Translation3d(16.580387, 5.548154, 1.451065),
                new Rotation3d(new Quaternion(0.000035, 0.000002, 0.000004, 1.000000)))));
    aprilTags.add(
        new AprilTag(
            5,
            new Pose3d(
                new Translation3d(14.701131, 8.204240, 1.355810),
                new Rotation3d(new Quaternion(0.707112, -0.000002, -0.000000, -0.707101)))));
    aprilTags.add(
        new AprilTag(
            6,
            new Pose3d(
                new Translation3d(1.842585, 8.205008, 1.355844),
                new Rotation3d(new Quaternion(0.707080, -0.000001, -0.000001, -0.707134)))));
    aprilTags.add(
        new AprilTag(
            7,
            new Pose3d(
                new Translation3d(-0.038100, 5.547868, 1.451102),
                new Rotation3d(new Quaternion(1.000000, -0.000000, 0.000000, 0.000000)))));
    aprilTags.add(
        new AprilTag(
            8,
            new Pose3d(
                new Translation3d(-0.037824, 4.982182, 1.451086),
                new Rotation3d(new Quaternion(1.000000, -0.000002, -0.000001, 0.000006)))));
    aprilTags.add(
        new AprilTag(
            9,
            new Pose3d(
                new Translation3d(0.356397, 0.883952, 1.355878),
                new Rotation3d(new Quaternion(0.866041, 0.000000, 0.000002, 0.499973)))));
    aprilTags.add(
        new AprilTag(
            10,
            new Pose3d(
                new Translation3d(1.462307, 0.246779, 1.355883),
                new Rotation3d(new Quaternion(0.866041, -0.000001, 0.000002, 0.499973)))));
    aprilTags.add(
        new AprilTag(
            11,
            new Pose3d(
                new Translation3d(11.905500, 3.713540, 1.320796),
                new Rotation3d(new Quaternion(0.866015, -0.000001, 0.000001, -0.500018)))));
    aprilTags.add(
        new AprilTag(
            12,
            new Pose3d(
                new Translation3d(11.904847, 4.498233, 1.320794),
                new Rotation3d(new Quaternion(0.866044, -0.000000, 0.000002, 0.499968)))));
    aprilTags.add(
        new AprilTag(
            13,
            new Pose3d(
                new Translation3d(11.220594, 4.104746, 1.320791),
                new Rotation3d(new Quaternion(0.000004, 0.000002, -0.000002, 1.000000)))));
    aprilTags.add(
        new AprilTag(
            14,
            new Pose3d(
                new Translation3d(5.320930, 4.105272, 1.320801),
                new Rotation3d(new Quaternion(1.000000, 0.000002, -0.000000, -0.000036)))));
    aprilTags.add(
        new AprilTag(
            15,
            new Pose3d(
                new Translation3d(4.642396, 4.499103, 1.320802),
                new Rotation3d(new Quaternion(0.499997, 0.000001, 0.000000, 0.866027)))));
    aprilTags.add(
        new AprilTag(
            16,
            new Pose3d(
                new Translation3d(4.642052, 3.714554, 1.320794),
                new Rotation3d(new Quaternion(-0.499967, 0.000001, 0.000001, 0.866044)))));
  }
}
