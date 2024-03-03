package frc.robot.subsystems.Intake.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs{        
        public double wristRads;
        public double wristRadsPerSec;

        public double wristVoltage;
        public double wristCurrent;

        public boolean wristHallEffect;
    }

    public default void updateInputs(WristIOInputs ioInputs){};
    public default void setVoltage(double voltage) {};
    public default void setAngle(double angle, double velocity) {};
    public default boolean hallEffectReset() {return false;};
    public default boolean getHallEffect(){return false;};
}
