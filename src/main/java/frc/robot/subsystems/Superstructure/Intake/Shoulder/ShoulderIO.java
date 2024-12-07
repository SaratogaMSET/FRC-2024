package frc.robot.subsystems.Superstructure.Intake.Shoulder;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {
  @AutoLog
  public static class ShoulderIOInputs {
    public double shoulderRads = 0.0;
    public double shoulderRadPerSecs = 0.0;

    public double shoulderVoltage;
    public double shoulderCurrent;
    public double motorShoulderRads = 0.0;
    public double motorRadPerSecs = 0.0;
  }

  /**
   * Updates inputs for shoulder voltage, current and angle in degrees
   *
   * @param inputs object of the class ActuatorShoulderIO for which values are updated for the
   *     shoulder
   */
  public default void updateInputs(ShoulderIOInputs inputs) {}
  ;

  /**
   * Sets shoulder voltage
   *
   * @param voltage double value for the power that the actuator (shoulder) is being set to
   */
  public default void setVoltage(double voltage) {}
  ;

  /**
   * Sets shoulder to a specific state based on angle and velocity
   *
   * @param angle double value for the position that the actuator (shoulder) is being set to
   * @param velocity double value for the speed that the actuator (shoulder) is being set to
   */
  public default void setAngle(double angle, double velocity) {}

  public default void setProfiled(double target, double FF) {}
}
