package frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    public ActuatorShoulderIOReal(){
        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();

        intakeTalonConfigs.Slot0.kP = 0.0;
        intakeTalonConfigs.Slot0.kI = 0.0;
        intakeTalonConfigs.Slot0.kD = 0.0;
        intakeTalonConfigs.Slot0.kV = 0.0;
        intakeTalonConfigs.CurrentLimits.StatorCurrentLimit = 25;// change later
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeTalonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeTalonOutputConfigs.DutyCycleNeutralDeadband = 0.0;
        intakeTalonOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);

        shoulder.getConfigurator().apply(intakeTalonConfigs);
    }
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

    public void setAngle(double angle, double velocity){
        double shoulderDegrees = 360 * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Intake.AcutatorConstants.SHOULDER_ENCODER_OFFSET);
        if (Math.abs(velocity) > 1)
            velocity = Math.signum(velocity);
        if (velocity < 0)
            velocity = 0;

    // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle

        angle = Math.min(AcutatorConstants.SHOULDER_HIGH_BOUND, Math.max(angle, AcutatorConstants.SHOULDER_LOW_BOUND));

        // Calculate gravity ff + PID
        double error = (angle - shoulderDegrees) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND);
        double gravity = ControlsConstants.k_G * Math.cos(shoulderDegrees + AcutatorConstants.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

        // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
        if (angle > shoulderDegrees) {
            shoulder.setVoltage((ControlsConstants.k_P * error * power) - gravity);
        } else {
            shoulder.setVoltage(((ControlsConstants.k_P * error) * power));
        }
    }

}
