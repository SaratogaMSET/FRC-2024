package frc.robot.subsystems.Intake.Wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
    @AutoLog
    public static class WristIOInputs{        
        public double rads;
        public double radsPerSec;

        public double voltage;
        public double current;

        public boolean hallEffect;
    }

    public default void updateInputs(WristIOInputs ioInputs){};
    public default void setVoltage(double voltage) {};
    public default void setAngle(double angle, double velocity) {};
    public default void hallEffectReset() {};
}
