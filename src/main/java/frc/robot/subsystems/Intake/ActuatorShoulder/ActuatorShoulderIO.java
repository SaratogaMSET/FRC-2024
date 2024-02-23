package frc.robot.subsystems.Intake.ActuatorShoulder;

import org.littletonrobotics.junction.AutoLog;

public interface ActuatorShoulderIO {
    @AutoLog
    public static class ActuatorShoulderIOInputs{
        public double shoulderDegrees = 0.0;
        public double shoulderAngVel = 0.0;
        // public double elevatorHeight = 0.0;
        public double shoulderVoltage;
        public double shoulderCurrent;
        // public ArmState armState = ArmState.NEUTRAL;
    }

    /**Updates inputs for shoulder voltage, current and angle in degrees
     * @param inputs object of the class ActuatorShoulderIO for which values are updated for the shoulder
    */
    public default void updateInputs(ActuatorShoulderIOInputs inputs) {};

    /**Sets shoulder voltage
     * @param voltage double value for the power that the actuator (shoulder) is being set to
    */
    public default void setVoltage(double voltage) {};

    /**Sets shoulder to a specific state based on angle and velocity
     * @param angle double value for the position that the actuator (shoulder) is being set to
     * @param velocity double value for the speed that the actuator (shoulder) is being set to
    */
    public default void setAngle(double angle, double velocity) {}
}

