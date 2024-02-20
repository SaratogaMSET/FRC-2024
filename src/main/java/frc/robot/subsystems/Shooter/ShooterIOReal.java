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
import frc.robot.Constants;

public class ShooterIOReal implements ShooterIO {
    TalonFX leftMotor = new TalonFX(Constants.ShooterConstants.kLeftMotorPort);
    TalonFX rightMotor = new TalonFX(Constants.ShooterConstants.kRightMotorPort);
    TalonFX angleMotor = new TalonFX(Constants.ShooterConstants.kAngleMotorPort);
    TalonFX feederMotor = new TalonFX(Constants.ShooterConstants.kFeederMotorPort);

    CANcoder encoder = new CANcoder(Constants.ShooterConstants.kEncoderPort);
    DigitalInput beamBreak = new DigitalInput(Constants.ShooterConstants.kBeamBreakPort);

    public ShooterIOReal(){
        configMotors();
    }
    public void configMotors(){
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

    public double angle(){
        return encoder.getAbsolutePosition().getValueAsDouble() - Constants.ShooterConstants.kEncoderOffset; 
    }
    public boolean beamBreak(){
        return beamBreak.get();
      }
    //TODO: motor RPS vs output RPS, if geared
    public double rpsLeft(){
        return leftMotor.getVelocity().getValueAsDouble();
    }
    public double rpsRight(){
        return rightMotor.getVelocity().getValueAsDouble();
    }
    public double rpsAvg(){
        return (rpsLeft() + rpsRight())/2;
    }
    public double rpsAngle(){
        return angleMotor.getVelocity().getValueAsDouble(); //TODO:Gearing
    }
    public double voltageLeft(){
        return leftMotor.getSupplyVoltage().getValueAsDouble();
    }
    public double voltageRight(){
        return rightMotor.getSupplyVoltage().getValueAsDouble();
    }
    public double voltageAngle(){
        return angleMotor.getSupplyVoltage().getValueAsDouble();
    }
    public boolean isRunning() {
        return Math.abs(rpsLeft()) + Math.abs(rpsRight()) > 0.1;
    }

    @Override
    public void setShooterVoltage(double voltage){
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setAnglerVoltage(double voltage){
        //TODO: Factor in velocity, if velocity will hit it in N control iterations, reduce by a factor based on how quickly it would hit based on current velocity
        //TODO: BOUNDS, FEEDFORWARD in NONPRIMITIVE
        if(angle() + rpsAngle() * 0.1 < Constants.ShooterConstants.kLowerBound && voltage < 0){
          voltage = 0;
        }
    
        if(angle() + rpsAngle() * 0.1 > Constants.ShooterConstants.kHigherBound && voltage > 0){
          voltage = 0;
        }
    
        angleMotor.setVoltage(voltage);
    }
    @Override
    public void setFeederVoltage(double voltage){
        feederMotor.setVoltage(voltage);
    }
}
