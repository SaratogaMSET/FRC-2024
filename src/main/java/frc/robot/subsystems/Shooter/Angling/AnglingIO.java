package frc.robot.subsystems.Shooter.Angling;

import org.littletonrobotics.junction.AutoLog;

public interface AnglingIO  {
    
    @AutoLog
    public static class AnglingIOInputs{
        public double pivotRad = 0;
        public double pivotRadPerSec = 0;
        public double pivotAppliedVolts = 0.0;
        public double pivotAppliedCurrent = 0.0;
    }

    public default void updateInputs(AnglingIOInputs inputs){};
    public default void setPivotVoltage(double voltage){};
}