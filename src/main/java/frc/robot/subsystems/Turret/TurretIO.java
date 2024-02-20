package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double phi;
        public double phiRadPerSec;

        public double voltage = 0.0;
        public double current = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs){}
    public default void setVoltage(double voltage){}
    public default void setDesiredPhi(double radians, double radiansPerSecond){}
}
