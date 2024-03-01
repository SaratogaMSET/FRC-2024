package frc.robot.subsystems.Intake.Wrist;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Intake.Wrist;

public class WristIOReal implements WristIO {
    CANSparkMax motor;
    DigitalInput hallEffect;
    static boolean previousHallEffect;

    public WristIOReal() {
        motor = new CANSparkMax(Wrist.MOTOR, MotorType.kBrushless);
        hallEffect = new DigitalInput(Wrist.HALL_EFFECT);
        previousHallEffect = false;
    }

    @Override
    /**
     * Updates inputs for wrist angle in degrees and the status of the hall effect
     * sensor
     */
    public void updateInputs(WristIOInputs inputs) {
        inputs.hallEffect = hallEffect.get();
        inputs.rads = 2 * Math.PI * (motor.getEncoder().getPosition() / Wrist.GEAR_RATIO);
        inputs.radsPerSec = 2 * Math.PI * (motor.getEncoder().getVelocity() / Wrist.GEAR_RATIO);
    }

    @Override
    /** Sets wrist voltage */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    /**
     * Resets wrist motor encoder if the wrist has just reached close to the sensor
     */
    public void hallEffectReset() {
        if (!previousHallEffect && hallEffect.get()) {
            setVoltage(0);
            //wristIOInputs.wristDegrees = AcutatorConstants.WRIST_ENCODER_HALL_EFFECT;
        }
        previousHallEffect = hallEffect.get();
    }
}