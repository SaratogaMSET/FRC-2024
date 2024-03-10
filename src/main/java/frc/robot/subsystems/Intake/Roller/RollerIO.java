package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO{
  @AutoLog
  public static class RollerIOInputs {
    public double intakeVoltage = 0.0;
    public double shooterVoltage = 0.0;
    public boolean intakeIR = false;
    public boolean shooterIR = false;
  }

  /**
   * Update the given inputs
   * @param inputs
   */
  public default void updateInputs(RollerIOInputs inputs) {};

  public default void setIntakeFeederVoltage(double voltage) {};

  public default void setShooterFeederVoltage(double voltage) {};
  public default void setShooterFeederMode(boolean brake) {};
  public default void setIntakeFeederMode(boolean brake) {};

}