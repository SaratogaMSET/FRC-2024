package frc.robot.subsystems.Intake.ActuatorWrist;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.Wrist;

public class ActuatorWristIOReal implements ActuatorWristIO{
    CANSparkMax wrist;
    DigitalInput input;

    public ActuatorWristIOReal(){
        wrist = new CANSparkMax(Wrist.MOTOR, MotorType.kBrushless);
        input = new DigitalInput(0);
    }

    @Override
    /**Updates inputs for wrist angle in degrees and the status of the hall effect sensor*/
    public void updateInputs(ActuatorWristIOInputs inputs) {
        double wristAngle = wrist.getEncoder().getPosition();

        inputs.hallEffects = input.get();
        inputs.wristDegrees = 360 * (wristAngle - Wrist.ENCODER_OFFSET);
    }

    @Override
    /**Sets wrist voltage*/
    public void setVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }
} 