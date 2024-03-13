package frc.robot;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;

public class MappedAprilTags {
    public AprilTagFieldLayout MappedLayout() {
        return new AprilTagFieldLayout(this.aprilTags, (26*12+7) * 0.0254, (54*12+1) * 0.0254);
    }

    List<AprilTag> aprilTags = new ArrayList<AprilTag>();

    public MappedAprilTags(){
        aprilTags.add(new AprilTag( 1, new Pose3d(new Translation3d( 15.067732, 0.255525, 1.337196), new Rotation3d(new Quaternion(0.499998, 0.000001, -0.000001, 0.866027)))));
        aprilTags.add(new AprilTag( 2, new Pose3d(new Translation3d( 16.173688, 0.888929, 1.337198), new Rotation3d(new Quaternion(0.499983, 0.000001, -0.000001, 0.866035)))));
        aprilTags.add(new AprilTag( 3, new Pose3d(new Translation3d( 16.567632, 4.987920, 1.432458), new Rotation3d(new Quaternion(0.000001, -0.000000, 0.000001, 1.000000)))));
        aprilTags.add(new AprilTag( 4, new Pose3d(new Translation3d( 16.567609, 5.553082, 1.432453), new Rotation3d(new Quaternion(-0.000014, 0.000001, -0.000001, 1.000000)))));
        aprilTags.add(new AprilTag( 5, new Pose3d(new Translation3d( 14.688848, 8.209086, 1.337203), new Rotation3d(new Quaternion(0.707109, 0.000000, 0.000000, -0.707105)))));
        aprilTags.add(new AprilTag( 6, new Pose3d(new Translation3d( 1.829681, 8.208489, 1.337210), new Rotation3d(new Quaternion(0.707119, -0.000000, -0.000000, -0.707095)))));
        aprilTags.add(new AprilTag( 7, new Pose3d(new Translation3d( -0.049807, 5.552617, 1.432464), new Rotation3d(new Quaternion(1.000000, 0.000000, 0.000000, 0.000000)))));
        aprilTags.add(new AprilTag( 8, new Pose3d(new Translation3d( -0.049783, 4.987414, 1.432469), new Rotation3d(new Quaternion(1.000000, 0.000001, 0.000000, -0.000000)))));
        aprilTags.add(new AprilTag( 9, new Pose3d(new Translation3d( 0.344637, 0.888381, 1.337218), new Rotation3d(new Quaternion(0.866019, 0.000000, 0.000000, 0.500012)))));
        aprilTags.add(new AprilTag( 10, new Pose3d(new Translation3d( 1.449862, 0.250347, 1.337218), new Rotation3d(new Quaternion(0.866013, -0.000000, 0.000001, 0.500022)))));
        aprilTags.add(new AprilTag( 11, new Pose3d(new Translation3d( 11.892974, 3.718066, 1.302159), new Rotation3d(new Quaternion(0.866026, -0.000001, 0.000000, -0.499999)))));
        aprilTags.add(new AprilTag( 12, new Pose3d(new Translation3d( 11.892921, 4.503107, 1.302152), new Rotation3d(new Quaternion(0.866019, -0.000001, 0.000001, 0.500010)))));
        aprilTags.add(new AprilTag( 13, new Pose3d(new Translation3d( 11.208759, 4.109976, 1.302158), new Rotation3d(new Quaternion(-0.000013, -0.000000, -0.000000, 1.000000)))));
        aprilTags.add(new AprilTag( 14, new Pose3d(new Translation3d( 5.309478, 4.109927, 1.302156), new Rotation3d(new Quaternion(1.000000, 0.000000, 0.000000, 0.000001)))));
        aprilTags.add(new AprilTag( 15, new Pose3d(new Translation3d( 4.629527, 4.502940, 1.302158), new Rotation3d(new Quaternion(0.499997, 0.000001, -0.000000, 0.866027)))));
        aprilTags.add(new AprilTag( 16, new Pose3d(new Translation3d( 4.629678, 3.718029, 1.302159), new Rotation3d(new Quaternion(0.500017, 0.000001, 0.000001, -0.866016)))));
    }
}
