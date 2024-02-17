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

    public default void updateInputs(ActuatorWristIOInputs ioInputs){};
    public default void setVoltage(double voltage) {};
    public default void setAngle(double angle, double velocity) {};
}
