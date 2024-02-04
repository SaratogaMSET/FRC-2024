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
    }

    public abstract void updateInputs(ActuatorWristIOInputs ioInputs);
    public void setVoltage(double voltage);
    public void setAngle(double angle);
}
