package frc.robot.subsystems.Turret;


import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class TurretIOReal implements TurretIO{

    public TurretIOReal(){
        configMotors();
    }
    TalonFX m_motor = new TalonFX(Constants.TurretConstants.kMotorPort);
    CANcoder encoder = new CANcoder(Constants.TurretConstants.kEncoderPort);
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
        m_motor.setInverted(false); //TODO:fix
        m_motor.setControl(new StaticBrake());
    }
    public double voltage(){
        return m_motor.getSupplyVoltage().getValueAsDouble();
    } 
    //TODO: Calibrate Zero Positions
    public double angle(){
        return encoder.getAbsolutePosition().getValueAsDouble() - Constants.TurretConstants.kEncoderOffset; 
    }
    public double angleDegrees(){
        return angle() * 180 / Math.PI;
      }
    //TOOD: Add gear ratio
    public double rps(){
        return m_motor.getVelocity().getValueAsDouble();
    }
    
    //TODO: FIX On Real with Regression
    public double[] maxAngleFromShooter(double shooterAngle){
        return new double[]{Constants.TurretConstants.kLowerBound, Constants.TurretConstants.kHigherBound};
      }
    public boolean[] speedCompensatedBounds(){
        double projection = angle() + rps() * 0.1;
        return new boolean[]{projection < Constants.TurretConstants.kLowerBound, projection > Constants.TurretConstants.kHigherBound};
    }

    public void updateInputs(TurretIOInputs inputs){} //TODO: What does this do

    @Override
    public void setVoltage(double voltage){
            //TODO: Tune RPS constant

        boolean[] boundsTriggered = speedCompensatedBounds();
        if(boundsTriggered[0] && voltage < 0){
            voltage = 0;
        }
        if(boundsTriggered[1] && voltage > 0){
            voltage = 0;
        }
    
        m_motor.setVoltage(voltage);
    
        this.voltage = voltage;
    }

    public void setDesiredPhi(double radians, double radiansPerSecond){} //TODO: not sure how this interacts
}
