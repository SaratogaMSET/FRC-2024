package frc.robot.CartographyOutput;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;

public class MappedAprilTagsBlue {
    public AprilTagFieldLayout MappedLayout() {
        return new AprilTagFieldLayout(this.aprilTags, 16.541, 8.211);
    }

    List<AprilTag> aprilTags = new ArrayList<AprilTag>();

    public MappedAprilTagsBlue(){
        aprilTags.add(new AprilTag( 6, new Pose3d(new Translation3d( 0.994927, 4.480389, 2.172094), new Rotation3d(new Quaternion(0.835677, -0.471917, 0.261988, -0.101497)))));
        aprilTags.add(new AprilTag( 7, new Pose3d(new Translation3d( -0.038100, 5.547868, 1.451102), new Rotation3d(new Quaternion(1.000000, -0.000000, 0.000000, 0.000000)))));
        aprilTags.add(new AprilTag( 8, new Pose3d(new Translation3d( -0.038100, 5.547868, 1.451102), new Rotation3d(new Quaternion(0.968364, -0.188578, -0.027608, -0.161085)))));
    }
}