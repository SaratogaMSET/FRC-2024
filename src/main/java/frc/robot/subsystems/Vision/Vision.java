package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.Logger;

public class Vision {

  private final VisionIO io;
  private final int index;
  public final VisionIOInputs inputs = new VisionIOInputs();

  public Vision(final VisionIO io) {
    this.io = io;
    this.index = io.index();
  }

  public double getIndex() {
    return index;
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs(Pose3d robotPose) {
    io.updateInputs(inputs, robotPose);
    Logger.processInputs("Vision/Camera " + String.valueOf(index), inputs);
  }
}
