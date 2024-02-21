package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs{
        public double[] shooterRPS = {0.0, 0.0};
        public double pivotRad = 0;
        public double pivotRadPerSec = 0;

        public double[] shooterAppliedVolts = {0.0, 0.0};
        public double[] shooterAppliedCurrent = {0.0, 0.0};

        public double pivotAppliedVolts = 0.0;
        public double pivotAppliedCurrent = 0.0;

        public double feederAppliedVolts = 0.0;
        public double feederAppliedCurrent = 0.0;

        public boolean beamBreakTriggered = false;
    }

    public default void updateInputs(ShooterIOInputs inputs){}
    public default void setDesiredAngler(double radians, double radiansPerSecond){}
    public default void setDesiredRPM(double RPM){}
    public default void setShooterVoltage(double voltage){}
    public default void setAnglerVoltage(double voltage){}
    public default void setFeederVoltage(double voltage){}
    public default void resetThetaEncoder(){}
    public default void setBeamBreak(boolean isTriggered){}
}
