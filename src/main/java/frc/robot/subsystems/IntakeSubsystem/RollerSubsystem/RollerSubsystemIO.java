package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSubsystemIO{
  @AutoLog
  public static class RollerSubsystemIOInputs {
    public double velocity = 0.0;
  }

  /**
   * @return
   */
  public abstract RollerSubsystemIOInputsAutoLogged
      updateInputs(); // FIXME: Check if this is still an error after



  /**
   * @return
   */
  public double getSpeed();

  /**
   * @param velocity
   */
  public void roll(double velocity);

   /**
   * @return
   */
  public boolean acquired();

   /**
   * @return
   */
  public boolean exited();


}