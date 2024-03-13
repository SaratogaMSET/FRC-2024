package frc.robot.CartographyOutput;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;

public class MappedAprilTagsRed {
    public AprilTagFieldLayout MappedLayout() {
        return new AprilTagFieldLayout(this.aprilTags, 16.541, 8.211);
    }

    List<AprilTag> aprilTags = new ArrayList<AprilTag>();

    public MappedAprilTagsRed(){
        aprilTags.add(new AprilTag( 1, new Pose3d(new Translation3d( 15.079304, 0.250322, 1.355838), new Rotation3d(new Quaternion(0.500027, 0.000000, -0.000001, 0.866010)))));
        aprilTags.add(new AprilTag( 2, new Pose3d(new Translation3d( 16.185023, 0.883902, 1.355847), new Rotation3d(new Quaternion(0.500004, 0.000000, -0.000000, 0.866023)))));
        aprilTags.add(new AprilTag( 3, new Pose3d(new Translation3d( 16.579295, 4.983232, 1.451095), new Rotation3d(new Quaternion(-0.000005, 0.000000, -0.000000, 1.000000)))));
        aprilTags.add(new AprilTag( 4, new Pose3d(new Translation3d( 16.579554, 5.547639, 1.451091), new Rotation3d(new Quaternion(0.000011, 0.000001, 0.000000, 1.000000)))));
        aprilTags.add(new AprilTag( 5, new Pose3d(new Translation3d( 14.701245, 8.203990, 1.355851), new Rotation3d(new Quaternion(0.707099, 0.000000, 0.000001, -0.707114)))));
        aprilTags.add(new AprilTag( 6, new Pose3d(new Translation3d( 1.841323, 8.204355, 1.355868), new Rotation3d(new Quaternion(0.707100, 0.000001, 0.000001, -0.707113)))));
        aprilTags.add(new AprilTag( 7, new Pose3d(new Translation3d( -0.038100, 5.547868, 1.451102), new Rotation3d(new Quaternion(1.000000, -0.000000, -0.000000, 0.000000)))));
        aprilTags.add(new AprilTag( 8, new Pose3d(new Translation3d( -0.038147, 4.982903, 1.451104), new Rotation3d(new Quaternion(1.000000, 0.000001, -0.000000, -0.000004)))));
        aprilTags.add(new AprilTag( 9, new Pose3d(new Translation3d( 0.356297, 0.884162, 1.355841), new Rotation3d(new Quaternion(0.866034, 0.000001, -0.000001, 0.499986)))));
        aprilTags.add(new AprilTag( 10, new Pose3d(new Translation3d( 1.461803, 0.246009, 1.355846), new Rotation3d(new Quaternion(0.866021, 0.000001, -0.000000, 0.500007)))));
        aprilTags.add(new AprilTag( 11, new Pose3d(new Translation3d( 11.904736, 3.713234, 1.320795), new Rotation3d(new Quaternion(0.866018, 0.000000, 0.000001, -0.500013)))));
        aprilTags.add(new AprilTag( 12, new Pose3d(new Translation3d( 11.904706, 4.498351, 1.320794), new Rotation3d(new Quaternion(0.866026, 0.000001, 0.000000, 0.499999)))));
        aprilTags.add(new AprilTag( 13, new Pose3d(new Translation3d( 11.220165, 4.104956, 1.320793), new Rotation3d(new Quaternion(0.000022, 0.000000, -0.000001, 1.000000)))));
        aprilTags.add(new AprilTag( 14, new Pose3d(new Translation3d( 5.320909, 4.105428, 1.320794), new Rotation3d(new Quaternion(1.000000, 0.000000, 0.000000, -0.000011)))));
        aprilTags.add(new AprilTag( 15, new Pose3d(new Translation3d( 4.641271, 4.498729, 1.320797), new Rotation3d(new Quaternion(0.500007, 0.000000, -0.000000, 0.866022)))));
        aprilTags.add(new AprilTag( 16, new Pose3d(new Translation3d( 4.641426, 3.713144, 1.320794), new Rotation3d(new Quaternion(0.500006, 0.000000, 0.000001, -0.866022)))));
    }
}
