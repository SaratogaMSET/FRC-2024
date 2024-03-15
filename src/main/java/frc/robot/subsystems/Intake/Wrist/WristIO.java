package frc.robot.subsystems.Intake.Wrist;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import frc.robot.Constants.Intake.Wrist;

public interface WristIO {
    public CANSparkMax motor = new CANSparkMax(Wrist.MOTOR, MotorType.kBrushless);
    @AutoLog
    public static class WristIOInputs{        
        public double wristRads;
        public double wristRadsPerSec;

        public double wristVoltage;
        public double wristCurrent;

        public boolean wristHallEffect;

        public boolean previouslyZeroed = false;
    }

    public default void updateInputs(WristIOInputs ioInputs){};
    public default void setVoltage(double voltage) {};
    public default void setAngle(double angle, double velocity) {};
    public default boolean hallEffectReset() {return false;};
    public default boolean getHallEffect(){return false;};
    public default void manualHallEffectReset(){};
}
