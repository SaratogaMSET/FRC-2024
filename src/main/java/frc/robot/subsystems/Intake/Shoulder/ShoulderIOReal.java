package frc.robot.subsystems.Intake.Shoulder;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.Shoulder;

public class ShoulderIOReal implements ShoulderIO {

    TalonFX motor = new TalonFX(Shoulder.MOTOR, Constants.CANBus);
    CANcoder encoder = new CANcoder(Shoulder.ENCODER,Constants.CANBus);

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

    public ShoulderIOReal(){
        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();
        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.2;
        motionMagicConfigs.MotionMagicAcceleration = 0.4;
        motionMagicConfigs.MotionMagicJerk = 0.6;

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
        inputs.motorShoulderRads = inputs.shoulderRads;
        inputs.motorRadPerSecs = motor.getVelocity().getValueAsDouble();
    }

    @Override
    /**Sets shoulder voltage*/
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    /**Sets shoulder with motion magic*/
    public void setProfiled(double target, double FF) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(target);
        motionMagicVoltage.withFeedForward(FF);
        motor.setControl(motionMagicVoltage);
    }

}
