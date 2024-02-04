package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.AcutatorConstants;

public class ActuatorWristIOReal implements ActuatorWristIO{
    CANSparkMax wrist;
    CANcoder wristEncoder;

    public ActuatorWristIOReal(){
        wrist = new CANSparkMax(AcutatorConstants.INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        wristEncoder = new CANcoder(AcutatorConstants.INTAKE_WRIST_ENCODER, "Placeholder");
    }
    @Override
    public void updateInputs(ActuatorWristIOInputs inputs) {
        inputs.wristDegrees = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Intake.AcutatorConstants.SHOULDER_ENCODER_OFFSET);
    }

    @Override
    public void setVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }
    @Override
    public void setAngle(double angle){
    }
} 