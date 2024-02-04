package frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.AcutatorConstants;
import frc.robot.Constants.Intake.AcutatorConstants.ControlsConstants;;

public class ActuatorShoulderIOReal implements ActuatorShoulderIO {
    TalonFX shoulder;
    CANcoder shoulderEncoder;

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

    PIDController controller = new PIDController(ControlsConstants.k_P, 0.0, ControlsConstants.k_D);
    @Override
    public void updateInputs(ActuatorShoulderIOInputs inputs) {
        inputs.shoulderDegrees = 360 * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Intake.AcutatorConstants.SHOULDER_ENCODER_OFFSET);
        inputs.shoulderVoltage = shoulder.getMotorVoltage().getValueAsDouble();
        inputs.shoulderCurrent = shoulder.getTorqueCurrent().getValueAsDouble();

    }

    @Override
    public void setVoltage(double voltage) {
        shoulder.setVoltage(voltage);
    }
    public void setAngle(double angle){}

}
