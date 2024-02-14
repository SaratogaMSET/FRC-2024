package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double phi;
        public double velocity;

        public double turretAppliedVolts = 0.0;
        public double turretAppliedCurrent = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs){}
    public default void setVoltage(double voltage){}
    public default void setDesiredPhi(double radians, double radiansPerSecond){}
}
