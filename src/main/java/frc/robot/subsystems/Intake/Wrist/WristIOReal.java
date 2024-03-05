package frc.robot.subsystems.Intake.Wrist;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Intake.Wrist;

public class WristIOReal implements WristIO {

    /* PLEASE NEVER CALL HALLEFFECT.GET(). YOU WOULD BE GREIFING. PLEASE CALL THE CLASS'S GETTER. THANK YOU */
    DigitalInput hallEffect = new DigitalInput(Wrist.HALL_EFFECT);
    static boolean previousHallEffect = false;
    double loopingOffset = 0.0;

    public WristIOReal() {
        motor.setSmartCurrentLimit(30);
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
            // inputs.wristRads = 0.0;
            // motor.getEncoder().setPosition(0.0);
            SmartDashboard.putNumber("find me haha", loopingOffset);
        }
        // else{
            inputs.wristRads = (2 * Math.PI * (motor.getEncoder().getPosition() / Wrist.GEAR_RATIO)) - Wrist.ENCODER_OFFSET+ 0.6 - loopingOffset;
        // }
        inputs.wristRadsPerSec = 2 * Math.PI * (motor.getEncoder().getVelocity() / Wrist.GEAR_RATIO);
        inputs.wristCurrent = motor.getOutputCurrent();
        inputs.wristVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    /** Sets wrist voltage */
    public void setVoltage(double voltage) {
        SmartDashboard.putNumber("haha find me 2.0 wrist motor voltage", voltage);
        // motor.setVoltage(voltage);
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
            test = true;
            // double newZeroPos = 0.15 / (2 * Math.PI) * Wrist.GEAR_RATIO;
            // motor.getEncoder().setPosition(0);
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