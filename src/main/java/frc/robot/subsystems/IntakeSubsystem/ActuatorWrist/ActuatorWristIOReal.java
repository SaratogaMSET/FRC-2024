package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.AcutatorConstants;

public class ActuatorWristIOReal implements ActuatorWristIO{
    CANSparkMax wrist;
    DigitalInput input;

    public ActuatorWristIOReal(){
        wrist = new CANSparkMax(AcutatorConstants.INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        input = new DigitalInput(0);
    }

    @Override
    public void updateInputs(ActuatorWristIOInputs inputs) {
        double wristAngle = wrist.getEncoder().getPosition();

        inputs.hallEffects = input.get();
        inputs.wristDegrees = 360 * (wristAngle - Intake.AcutatorConstants.WRIST_ENCODER_OFFSET);
    }

    @Override
    public void setVoltage(double voltage) {
        wrist.setVoltage(voltage);
    }
} 