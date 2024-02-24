package frc.robot.subsystems.Intake.ActuatorWrist;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Intake.Wrist;

public class ActuatorWristIOReal implements ActuatorWristIO {
    CANSparkMax wrist;
    DigitalInput hallEffect;
    static boolean previousHallEffect;

    public ActuatorWristIOReal() {
        wrist = new CANSparkMax(Wrist.MOTOR, MotorType.kBrushless);
        hallEffect = new DigitalInput(Wrist.HALL_EFFECT);
        previousHallEffect = false;
    }

    @Override
    /**
     * Updates inputs for wrist angle in degrees and the status of the hall effect
     * sensor
     */
    public void updateInputs(ActuatorWristIOInputs inputs) {
        double wristAngle = wrist.getEncoder().getPosition();

        inputs.hallEffect = hallEffect.get();
        inputs.wristDegrees = 360 * (wristAngle - Wrist.ENCODER_OFFSET);
    }

    @Override
    /** Sets wrist voltage */
    public void setVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }

    @Override
    /**
     * Resets wrist motor encoder if the wrist has just reached close to the sensor
     */
    public void hallEffectReset() {
        if (!previousHallEffect && hallEffect.get()) {
            wrist.set(0.0);
            // wristIOInputs.wristDegrees = AcutatorConstants.WRIST_ENCODER_HALL_EFFECT; // TODO: WHAT is this code supposed to do? (Answer: we're using the hall effect's rising edge to reset the encoder)
        }
        previousHallEffect = hallEffect.get();
    }
}