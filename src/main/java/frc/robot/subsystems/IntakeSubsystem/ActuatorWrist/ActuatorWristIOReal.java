package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.AcutatorConstants;
import frc.robot.Constants.Intake.AcutatorConstants.ControlsConstants;

public class ActuatorWristIOReal implements ActuatorWristIO{
    CANSparkMax wrist;
    CANcoder wristEncoder;

    public ActuatorWristIOReal(){
        wrist = new CANSparkMax(AcutatorConstants.INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        wristEncoder = new CANcoder(AcutatorConstants.INTAKE_WRIST_ENCODER, "Placeholder");
    }
    @Override
    public void updateInputs(ActuatorWristIOInputs inputs) {
        inputs.wristDegrees = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Intake.AcutatorConstants.WRIST_ENCODER_OFFSET);
    }

    @Override
    public void setVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }
    @Override
     public void setAngle(double angle, double velocity){
        double wristDegrees = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Intake.AcutatorConstants.SHOULDER_ENCODER_OFFSET);
        if (Math.abs(velocity) > 1)
        velocity = Math.signum(velocity);
        if (velocity < 0)
        velocity = 0;

    // Calculate the voltage draw 
    double power = 12 * Math.abs(velocity);

    // Enforce bounds on angle

    angle = Math.min(AcutatorConstants.SHOULDER_HIGH_BOUND, Math.max(angle, AcutatorConstants.SHOULDER_LOW_BOUND));

    // Calculate gravity ff + PID
    double error = (angle - wristDegrees) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND);
    double gravity = ControlsConstants.k_G * Math.cos(wristDegrees + AcutatorConstants.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

    // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
    if (angle > wristDegrees) {
        wrist.setVoltage((ControlsConstants.k_P * error * power) - gravity);
    } else {
        wrist.setVoltage(((ControlsConstants.k_P * error) * power));
    }
    }
} 