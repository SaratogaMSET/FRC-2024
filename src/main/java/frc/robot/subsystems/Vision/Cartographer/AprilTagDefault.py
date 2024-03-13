import gtsam
import math

# https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
april_tag_poses = {
    1: gtsam.Pose3(gtsam.Rot3.Rz(120*math.pi/180), gtsam.Point3(593.68 * 0.0254, 9.86 * 0.0254, 53.38 * 0.0254)),
    2: gtsam.Pose3(gtsam.Rot3.Rz(120*math.pi/180), gtsam.Point3(637.21 * 0.0254, 34.79 * 0.0254, 53.38 * 0.0254)),

    3: gtsam.Pose3(gtsam.Rot3.Rz(180*math.pi/180), gtsam.Point3(652.73 * 0.0254, 196.17 * 0.0254, 57.13 * 0.0254)),
    4: gtsam.Pose3(gtsam.Rot3.Rz(180*math.pi/180), gtsam.Point3(652.73 * 0.0254, 218.42 * 0.0254, 57.13 * 0.0254)),

    5: gtsam.Pose3(gtsam.Rot3.Rz(270*math.pi/180), gtsam.Point3(578.77 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254)),
    6: gtsam.Pose3(gtsam.Rot3.Rz(270*math.pi/180), gtsam.Point3(72.5 * 0.0254, 323.00 * 0.0254, 53.38 * 0.0254)),

    7: gtsam.Pose3(gtsam.Rot3.Rz(0*math.pi/180), gtsam.Point3(-1.50 * 0.0254, 218.42 * 0.0254, 57.13 * 0.0254)),
    8: gtsam.Pose3(gtsam.Rot3.Rz(0*math.pi/180), gtsam.Point3(-1.50 * 0.0254, 196.17 * 0.0254, 57.13 * 0.0254)),

    9: gtsam.Pose3(gtsam.Rot3.Rz(60*math.pi/180), gtsam.Point3(14.02 * 0.0254, 34.79 * 0.0254, 53.38 * 0.0254)),
    10: gtsam.Pose3(gtsam.Rot3.Rz(60*math.pi/180), gtsam.Point3(57.54 * 0.0254, 9.68 * 0.0254, 53.38 * 0.0254)),

    11: gtsam.Pose3(gtsam.Rot3.Rz(300*math.pi/180), gtsam.Point3(468.69 * 0.0254, 146.19 * 0.0254, 52.00 * 0.0254)),
    12: gtsam.Pose3(gtsam.Rot3.Rz(60*math.pi/180), gtsam.Point3(468.69 * 0.0254, 177.10 * 0.0254, 52.00 * 0.0254)),

    13: gtsam.Pose3(gtsam.Rot3.Rz(180*math.pi/180), gtsam.Point3(441.74 * 0.0254, 161.62 * 0.0254, 52.00 * 0.0254)),
    14: gtsam.Pose3(gtsam.Rot3.Rz(0*math.pi/180), gtsam.Point3(209.48 * 0.0254, 161.62 * 0.0254, 52.00 * 0.0254)),

    15: gtsam.Pose3(gtsam.Rot3.Rz(120*math.pi/180), gtsam.Point3(182.73 * 0.0254, 177.10 * 0.0254, 52.00 * 0.0254)),
    16: gtsam.Pose3(gtsam.Rot3.Rz(240*math.pi/180), gtsam.Point3(182.73 * 0.0254, 146.19 * 0.0254, 52.00 * 0.0254)),
}
