package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ShoulderSubsystem;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;

public interface ShoulderSubsystemIO {

    @AutoLog
    public static class ShoulderSubsystemIOInputs{
        public double shoulderDegrees = 0.0;
        public double shoulderCurrent = 0.0;
        public double shoulderVoltage = 0.0;
    }

    /**
   * Update the given inputs
   * @param inputs
   */
  public default void updateInputs(ShoulderSubsystemIOInputs ioInputs){}

  /**
   * @param angle in degrees, where the horizontal is 0
   * @param velocity within the interval [0, 1]
   */
  public default void setAngle(double angle, double speedMagnitude){}


  /**
   * @param voltage within the interval [0, 1]
   */
  public default void setVoltage(double voltage){}

}
