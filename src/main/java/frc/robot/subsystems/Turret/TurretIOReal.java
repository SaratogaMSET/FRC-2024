package frc.robot.subsystems.Turret;


import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class TurretIOReal implements TurretIO{

    public TurretIOReal(){
        configMotors();
    }
    TalonFX m_motor = new TalonFX(Constants.TurretConstants.kMotorPort, Constants.CANBus);
    CANcoder encoder = new CANcoder(Constants.TurretConstants.kEncoderPort, Constants.CANBus);
    double voltage = 0;
    
    public void configMotors(){
        TalonFXConfiguration generalConfig = new TalonFXConfiguration();
        MotorOutputConfigs motorConfig = new MotorOutputConfigs();
        ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

        motorConfig.withPeakForwardDutyCycle(1);
        motorConfig.withPeakReverseDutyCycle(1);
        
        voltageRampConfig.withVoltageClosedLoopRampPeriod(0.5);

        currentLimitConfig.withStatorCurrentLimit(5);
        currentLimitConfig.withStatorCurrentLimitEnable(true);

        generalConfig.withMotorOutput(motorConfig);
        generalConfig.withClosedLoopRamps(voltageRampConfig);
        generalConfig.withCurrentLimits(currentLimitConfig);

        m_motor.getConfigurator().apply(generalConfig);
        m_motor.setInverted(true);
        // m_motor.setControl(new CoastOut());
        m_motor.setControl(new StaticBrake());
    }

    public void updateInputs(TurretIOInputs inputs){
        inputs.turretRad = 2 * Math.PI * (encoder.getAbsolutePosition().getValueAsDouble() - Constants.TurretConstants.kEncoderOffset);
        inputs.turretRadPerSec = m_motor.getVelocity().getValueAsDouble() * 2 * Math.PI / TurretConstants.kMotorGearing;
        inputs.turretVoltage = m_motor.getSupplyVoltage().getValueAsDouble();
        inputs.turretCurrent = m_motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }
}
