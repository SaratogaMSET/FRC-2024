import SimDataLoader
import LogDataLoader
from GTSAMCalib import *
from datetime import datetime

def relocalize_tags(landmark_poses, tag_id: int):
    all_tag_transform = landmark_poses[tag_id].between(april_tag_initial[7])
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
    with open(os.path.join(script_dir, output_file), 'w') as f:
        json.dump(output_data, f, indent=2)
        
def main(print_java_layout: bool, print_error: bool, write_to_json: bool, file_name: str = "TestLog.csv"):
    # optimized_landmark_poses = optimize_landmark_poses(SimDataLoader.simulateData(2000))
    optimized_landmark_poses = optimize_landmark_poses(LogDataLoader.processLog(file_name))

    localize_to_blue = True #Blue

    globalized_landmark_poses = relocalize_tags(optimized_landmark_poses, 7 if localize_to_blue else 4)
    for tag_id, pose in globalized_landmark_poses.items():
        translation = list(pose.translation())
        rotation = pose.rotation().toQuaternion()
        quaternion = [rotation.w(), rotation.x(), rotation.y(), rotation.z()]

        if(print_java_layout):
            translation_str = ', '.join(['{:.6f}'.format(value) for value in translation])
            rotation_str = ', '.join(['{:.6f}'.format(value) for value in quaternion])
            if(tag_id > 0):
                print(f"aprilTags.add(new AprilTag( {tag_id}, new Pose3d(new Translation3d( {translation_str}), new Rotation3d(new Quaternion({rotation_str})))));")

    if(print_error):
        errors = calculate_error(globalized_landmark_poses, april_tag_initial)
        for tag_id, error in errors.items():
            translation_error, rotation_error = error
            print(f"Tag {tag_id} Deviation in inches and degrees: Translation = {translation_error / 0.0254}, Rotation = {rotation_error * 180/math.pi}")

    output_file = f'relocalized{datetime.now().strftime("%Y-%m-%d-%H:%M:%S")}.json'
    
    if(write_to_json):
        write_tag_poses_to_json(globalized_landmark_poses, output_file)

if __name__ == "__main__":
    # the .csv log file must be in the same folder for now
    main(True, True, False)