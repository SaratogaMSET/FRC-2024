package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ShooterConstants;
  
public class ShooterIOReal implements ShooterIO{
    TalonFX leftMotor = new TalonFX(ShooterConstants.kLeftMotorPort);;
    TalonFX rightMotor = new TalonFX(ShooterConstants.kRightMotorPort);
    TalonFX angleMotor = new TalonFX(ShooterConstants.kAngleMotorPort);
    TalonFX feederMotor = new TalonFX(ShooterConstants.kFeederMotorPort);

    CANcoder encoder = new CANcoder(ShooterConstants.kEncoderPort);
    DigitalInput beamBreak = new DigitalInput(ShooterConstants.kBeamBreakPort);

    public ShooterIOReal(){
        TalonFXConfiguration generalConfig = new TalonFXConfiguration();
        MotorOutputConfigs motorConfig = new MotorOutputConfigs();
        ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

        motorConfig.withPeakForwardDutyCycle(1);
        motorConfig.withPeakReverseDutyCycle(1);
        
        voltageRampConfig.withVoltageClosedLoopRampPeriod(1);

        currentLimitConfig.withStatorCurrentLimit(40); //TODO: Fix/TUNE
        currentLimitConfig.withStatorCurrentLimitEnable(true);

        generalConfig.withMotorOutput(motorConfig);
        generalConfig.withClosedLoopRamps(voltageRampConfig);
        generalConfig.withCurrentLimits(currentLimitConfig);

        leftMotor.getConfigurator().apply(generalConfig);
        rightMotor.getConfigurator().apply(generalConfig);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
        leftMotor.setControl(new CoastOut());
        rightMotor.setControl(new CoastOut());

        feederMotor.setInverted(false);
        angleMotor.setInverted(false);
        feederMotor.setControl(new CoastOut());
        angleMotor.setControl(new StaticBrake());

    }
    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.shooterRPS = new double[]{leftMotor.getVelocity().getValueAsDouble(), rightMotor.getVelocity().getValueAsDouble()};

        inputs.theta = encoder.getAbsolutePosition().getValueAsDouble() - ShooterConstants.kEncoderOffset;
        inputs.thetaRadPerSec = angleMotor.getVelocity().getValueAsDouble() * 2 * Math.PI; //TODO: Add gear ratio

        inputs.shooterAppliedVolts = new double[]{leftMotor.getSupplyVoltage().getValueAsDouble(), rightMotor.getSupplyVoltage().getValueAsDouble()};
        inputs.shooterAppliedCurrent = new double[]{leftMotor.getStatorCurrent().getValueAsDouble(), rightMotor.getStatorCurrent().getValueAsDouble()};

        inputs.anglerAppliedVolts = angleMotor.getSupplyVoltage().getValueAsDouble();
        inputs.anglerAppliedCurrent = angleMotor.getStatorCurrent().getValueAsDouble();

        inputs.feederAppliedVolts = feederMotor.getSupplyVoltage().getValueAsDouble();
        inputs.feederAppliedCurrent = feederMotor.getStatorCurrent().getValueAsDouble();

        inputs.beamBreakTriggered = beamBreak.get();
    }

    @Override
    public void setShooterVoltage(double voltage){
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setAnglerVoltage(double voltage){
        angleMotor.setVoltage(voltage);
    }
    @Override
    public void setFeederVoltage(double voltage){
        feederMotor.setVoltage(voltage);
    }
    
    @Override
    public void setDesiredAngler(double radians, double radiansPerSecond){}

    @Override
    public void setDesiredRPM(double RPM){}

    @Override
    public void resetThetaEncoder(){}

    @Override
    public void setBeamBreak(boolean isTriggered){}
}
