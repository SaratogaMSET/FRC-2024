package frc.robot.subsystems.Intake.RollerSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO{
  @AutoLog
  public static class RollerIOInputs {
    public double velocity = 0.0;
    public boolean rollerIR = false;
  }

  /**
   * Update the given inputs
   * @param inputs
   */
  public default void updateInputs(RollerIOInputs inputs) {};

  public default void setVoltage(double voltage) {};

}