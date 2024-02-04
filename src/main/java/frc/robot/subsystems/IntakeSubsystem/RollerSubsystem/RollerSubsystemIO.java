package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSubsystemIO{
  @AutoLog
  public static class RollerSubsystemIOInputs {
    public double velocity = 0.0;
    public boolean wristIR = false;
    public boolean shooterIR = false;
    public int ringCount = 0; // TODO: write after prototype
  }

  /**
   * Update the given inputs
   * @param inputs
   */
  public abstract void updateInputs(RollerSubsystemIOInputs inputs);

  /**
   * @return the percentage velocity of the roller
   */
  public double getVelocity();

  /**
   * Set the roller to roll at a velocity within the interval [-1, 1]
   * @param velocity
   */
  public void roll(double velocity);

   /**
   * @return the value of the roller IR gate
   */
  public boolean wristIR();

   /**
   * @return the value of the shooter IR gate
   */
  public boolean shooterIR();


}