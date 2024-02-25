package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs{
        public double turretRad;
        public double turretRadPerSec;

        public double turretVoltage = 0.0;
        public double turretCurrent = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs){}
    public default void setVoltage(double voltage){}
}