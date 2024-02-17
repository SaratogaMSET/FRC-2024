package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.AcutatorConstants;

public class ActuatorWristIOReal implements ActuatorWristIO{
    CANSparkMax wrist;
    CANcoder wristEncoder;
    DigitalInput input;

    public ActuatorWristIOReal(){
        wrist = new CANSparkMax(AcutatorConstants.INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        wristEncoder = new CANcoder(AcutatorConstants.INTAKE_WRIST_ENCODER, "Placeholder");
        input = new DigitalInput(0);
    }

    @Override
    public void updateInputs(ActuatorWristIOInputs inputs) {
        inputs.wristDegrees = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Intake.AcutatorConstants.WRIST_ENCODER_OFFSET);
        inputs.hallEffects = input.get();
    }

    @Override
    public void setVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }
} 