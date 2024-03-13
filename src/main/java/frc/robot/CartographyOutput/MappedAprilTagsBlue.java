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
        aprilTags.add(new AprilTag( 6, new Pose3d(new Translation3d( 0.000184, 5.380489, 1.248368), new Rotation3d(new Quaternion(0.756581, -0.512161, 0.124377, 0.387048)))));
        aprilTags.add(new AprilTag( 7, new Pose3d(new Translation3d( -0.038100, 5.547868, 1.451102), new Rotation3d(new Quaternion(1.000000, -0.000000, 0.000000, 0.000000)))));
        aprilTags.add(new AprilTag( 8, new Pose3d(new Translation3d( -0.047127, 5.549344, 1.446903), new Rotation3d(new Quaternion(0.993766, 0.058415, -0.002337, -0.094929)))));
    }
}