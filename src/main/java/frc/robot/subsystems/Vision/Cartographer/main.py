import SimDataLoader
import LogDataLoader
from GTSAMCalib import *
from datetime import datetime

def relocalize_tags(landmark_poses, tag_id: int):
    all_tag_transform = landmark_poses[tag_id].between(april_tag_initial[tag_id])
    #print(all_tag_transform.compose(optimized_landmark_poses[7]).translation(), april_tag_initial[7].translation())
    all_tag_transform = gtsam.Pose3(all_tag_transform.rotation(), all_tag_transform.translation() - all_tag_transform.compose(landmark_poses[tag_id]).translation() + april_tag_initial[tag_id].translation())

    globalized_landmark_poses = {}
    for tag_id, pose in landmark_poses.items():
        # print(f"Tag {tag_id}:")
        # print(pose)

        globalized_pose = all_tag_transform.compose(pose)
        globalized_landmark_poses[tag_id] = globalized_pose

    return globalized_landmark_poses

def write_tag_poses_to_json(optimized_landmark_poses, output_file):
    tags_data = []
    for tag_id, pose in optimized_landmark_poses.items():
        if(tag_id > 0):
            translation = {"x": pose.x(), "y": pose.y(), "z": pose.z()}
            rotation_quaternion = {"W": pose.rotation().toQuaternion().w(),
                                "X": pose.rotation().toQuaternion().x(),
                                "Y": pose.rotation().toQuaternion().y(),
                                "Z": pose.rotation().toQuaternion().z()}
            pose_data = {"ID": tag_id,
                        "pose": {"translation": translation,
                                "rotation": {"quaternion": rotation_quaternion}}}
            tags_data.append(pose_data)

    output_data = {"tags": tags_data}
    output_data["field"] = {
        "length": 16.541,
        "width": 8.211
    }
        
    script_dir = os.path.dirname(os.path.abspath(__file__))
    self_path = os.path.join(script_dir, output_file)
    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=2)
        
def write_to_java(tags, output_file, color):
    java_code = """package frc.robot.CartographyOutput;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;

public class MappedAprilTags""" + color + """ {
    public AprilTagFieldLayout MappedLayout() {
        return new AprilTagFieldLayout(this.aprilTags, 16.541, 8.211);
    }

    List<AprilTag> aprilTags = new ArrayList<AprilTag>();

    public MappedAprilTags""" + color + """(){
"""
    for tag_id, pose in tags.items():
        if(tag_id > 0):
            quaternion = [pose.rotation().toQuaternion().w(), pose.rotation().toQuaternion().x(), pose.rotation().toQuaternion().y(), pose.rotation().toQuaternion().z()]
            translation_str = ", ".join([f"{val:.6f}" for val in list(pose.translation())])
            rotation_str = ", ".join([f"{val:.6f}" for val in quaternion])
            
            java_code += f"""        aprilTags.add(new AprilTag( {tag_id}, new Pose3d(new Translation3d( {translation_str}), new Rotation3d(new Quaternion({rotation_str})))));\n"""

    java_code += """    }
}"""
        
    with open(output_file, "w") as file:
        file.write(java_code)
def main(output_java: bool, output_json: bool, print_error: bool, write_blue: bool, write_red: bool, file_name: str = "TestLog.csv"):
    # optimized_landmark_poses = optimize_landmark_poses(SimDataLoader.simulateData(2000))
    optimized_landmark_poses = optimize_landmark_poses(LogDataLoader.processLog(file_name))

    # print(optimized_landmark_poses)

    blue_landmark_poses = None
    red_landmark_poses = None
    if(write_blue):
        blue_landmark_poses = relocalize_tags(optimized_landmark_poses, 7)
    if(write_red):
        red_landmark_poses = relocalize_tags(optimized_landmark_poses, 4)

    blue_output_java = 'MappedAprilTagsBlue.java'
    red_output_java = 'MappedAprilTagsRed.java'

    if(output_java):
            if(write_blue):
                write_to_java(blue_landmark_poses, blue_output_java, "Blue")
            if(write_red):
                write_to_java(red_landmark_poses, red_output_java, "Red")

    if(print_error):
        errors = calculate_error(blue_landmark_poses, april_tag_initial)
        for tag_id, error in errors.items():
            translation_error, rotation_error = error
            print(f"Tag {tag_id} Deviation in inches and degrees: Translation = {translation_error / 0.0254}, Rotation = {rotation_error * 180/math.pi}")

    blue_output_json = 'BlueAprilTags.json'
    red_output_json = 'RedAprilTags.json'
    
    if(output_json):
        if(write_blue):
            write_tag_poses_to_json(blue_landmark_poses, blue_output_json)
        if(write_red):
            write_tag_poses_to_json(red_landmark_poses, red_output_json)

if __name__ == "__main__":
    # the .csv log file must be in the same folder for now
    main(output_java=True, output_json=True, print_error=True, write_blue=True, write_red=False)