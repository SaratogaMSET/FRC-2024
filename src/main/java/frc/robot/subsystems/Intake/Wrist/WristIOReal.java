package frc.robot.subsystems.Intake.Wrist;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Intake.Wrist;

public class WristIOReal implements WristIO {

    /* PLEASE NEVER CALL HALLEFFECT.GET(). YOU WOULD BE GREIFING. PLEASE CALL THE CLASS'S GETTER. THANK YOU */
    CANSparkMax motor = new CANSparkMax(Wrist.MOTOR, MotorType.kBrushless);
    DigitalInput hallEffect = new DigitalInput(Wrist.HALL_EFFECT);
    static boolean previousHallEffect = false;
    double loopingOffset = 0.0;

    public WristIOReal() {
        motor.setSmartCurrentLimit(20);
    }

    @Override
    /**
     * Updates inputs for wrist angle in degrees and the status of the hall effect
     * sensor
     */
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristHallEffect = getHallEffect(); // Returns true if the sensor senses the wrist!!!!! 
        if (hallEffectReset()){
            loopingOffset += inputs.wristRads;
            SmartDashboard.putNumber("find me haha", loopingOffset);
        }
        inputs.wristRads = (2 * Math.PI * (motor.getEncoder().getPosition() / Wrist.GEAR_RATIO)) - Wrist.ENCODER_OFFSET - loopingOffset;
        inputs.wristRadsPerSec = 2 * Math.PI * (motor.getEncoder().getVelocity() / Wrist.GEAR_RATIO);
        inputs.wristCurrent = motor.getOutputCurrent();
        inputs.wristVoltage = motor.getAppliedOutput() * 12.0;
    }

    @Override
    /** Sets wrist voltage */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    /**
     * 
     * Dude what does this do. 
     * Resets wrist motor encoder if the wrist has just reached close to the sensor
     */
    public boolean hallEffectReset() {
        boolean test = false;
        if (!previousHallEffect && getHallEffect()) { // If previous = false and current = true, we can reset hall effect. Returns true. 
            setVoltage(0);
            test = true;
            //wristIOInputs.wristDegrees = AcutatorConstants.WRIST_ENCODER_HALL_EFFECT;
        }
        previousHallEffect = getHallEffect();
        return test;
    }

    @Override
    public boolean getHallEffect(){
        return !hallEffect.get();
    }
}