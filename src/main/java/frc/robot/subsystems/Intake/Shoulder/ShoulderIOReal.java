package frc.robot.subsystems.Intake.Shoulder;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.Shoulder;

public class ShoulderIOReal implements ShoulderIO {
    TalonFX motor = new TalonFX(Shoulder.MOTOR, Constants.canbus);
    CANcoder encoder = new CANcoder(Shoulder.ENCODER,Constants.canbus);

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

    public ShoulderIOReal(){
        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();

        intakeTalonConfigs.CurrentLimits.StatorCurrentLimit = 30;// change later
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeTalonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeTalonOutputConfigs.DutyCycleNeutralDeadband = 0.0;
        intakeTalonOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);

        motor.getConfigurator().apply(intakeTalonConfigs);
        motor.setInverted(true);
    }
    @Override
     /**Updates inputs for shoulder voltage, current and angle*/
    public void updateInputs(ShoulderIOInputs inputs) {
        inputs.shoulderVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.shoulderCurrent = motor.getTorqueCurrent().getValueAsDouble();
        inputs.shoulderRads = -2 * Math.PI * (encoder.getAbsolutePosition().getValueAsDouble() - Shoulder.ENCODER_OFFSET);
        inputs.shoulderRadPerSecs = -2 * Math.PI * motor.getVelocity().getValueAsDouble() / Shoulder.GEAR_RATIO;
    }

    @Override
    /**Sets shoulder voltage*/
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

}
