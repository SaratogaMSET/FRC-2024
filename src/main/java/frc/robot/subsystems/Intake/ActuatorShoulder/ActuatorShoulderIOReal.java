package frc.robot.subsystems.Intake.ActuatorShoulder;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.Intake;

public class ActuatorShoulderIOReal implements ActuatorShoulderIO {
    TalonFX shoulder;
    CANcoder shoulderEncoder;

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

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
     /**Updates inputs for shoulder voltage, current and angle*/
    public void updateInputs(ActuatorShoulderIOInputs inputs) {
        inputs.shoulderVoltage = shoulder.getMotorVoltage().getValueAsDouble();
        inputs.shoulderCurrent = shoulder.getTorqueCurrent().getValueAsDouble();
        double shoulderAngle = shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Intake.Shoulder.ENCODER_OFFSET;

        inputs.shoulderDegrees = 360 * (shoulderAngle) + Intake.Shoulder.ENCODER_OFFSET_FROM_ZERO;
    }

    @Override
    /**Sets shoulder voltage*/
    public void setVoltage(double voltage) {
        shoulder.setVoltage(voltage);
    }

}
