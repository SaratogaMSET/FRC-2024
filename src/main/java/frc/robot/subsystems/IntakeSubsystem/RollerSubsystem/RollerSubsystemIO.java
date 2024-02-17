package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSubsystemIO{
  @AutoLog
  public static class RollerSubsystemIOInputs {
    public double velocity = 0.0;
    public boolean rollerIR = false;
    public boolean shooterIR = false;
    public int ringCount = 0; // TODO: write after prototype
  }

  /**
   * Update the given inputs
   * @param inputs
   */
  public default void updateInputs(RollerSubsystemIOInputs inputs) {};

  /**
   * Set the roller to roll at a velocity within the interval [-1, 1]
   * @param velocity
   */
  public default void roll(double velocity) {};

}