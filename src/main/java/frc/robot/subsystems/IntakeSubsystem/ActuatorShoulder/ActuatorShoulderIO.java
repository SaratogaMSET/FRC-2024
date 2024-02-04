package frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder;

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

    public abstract void updateInputs(ActuatorShoulderIOInputs inputs);
    public void setVoltage(double voltage);
    public void setAngle(double angle);
}

