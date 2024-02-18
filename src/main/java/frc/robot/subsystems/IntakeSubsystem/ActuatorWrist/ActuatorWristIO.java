package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import org.littletonrobotics.junction.AutoLog;

public interface ActuatorWristIO {
    @AutoLog
    public static class ActuatorWristIOInputs{        
        public double wristDegrees = 0.0;
        public double wristAngVel = 0.0;
        // public double elevatorHeight = 0.0;
        public double wristVoltage;
        public double wristCurrent;
        // public ArmState armState = ArmState.NEUTRAL;
        public boolean hallEffects;
    }

    /**Updates inputs for wrist angle in degrees and the status of the hall effect sensor
     * @param inputs object of the class ActuatorWristIOInputs for which values are updated for the wrist
    */
    public default void updateInputs(ActuatorWristIOInputs ioInputs){};

     /**Sets wrist voltage
     * @param voltage double value for the power that the actuator (wrist) is being set to
    */
    public default void setVoltage(double voltage) {};

    /**Sets shoulder to a specific state based on angle and velocity
     * @param angle double value for the position that the actuator (wrist) is being set to
     * @param velocity double value for the speed that the actuator (wrist) is being set to
    */
    public default void setAngle(double angle, double velocity) {};
}
