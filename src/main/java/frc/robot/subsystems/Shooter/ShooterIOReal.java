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
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
  
public class ShooterIOReal implements ShooterIO{
    TalonFX leftMotor = new TalonFX(ShooterConstants.kLeftMotorPort);;
    TalonFX rightMotor = new TalonFX(ShooterConstants.kRightMotorPort);
    TalonFX angleMotor = new TalonFX(ShooterConstants.kAngleMotorPort);
    TalonFX feederMotor = new TalonFX(ShooterConstants.kFeederMotorPort);

    CANcoder encoder = new CANcoder(ShooterConstants.kEncoderPort);
    DigitalInput beamBreak = new DigitalInput(ShooterConstants.kBeamBreakPort);

    double previousTime = Timer.getFPGATimestamp();
    double previousAngle = angleRad();

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
        inputs.shooterRPM = rpsAvg() * 60;
        inputs.theta = angleRad();
        
        inputs.thetaPerSeconds = angleRadPerSec();

        inputs.shooterAppliedVolts = new double[]{voltageLeft(), voltageRight()};
        inputs.shooterAppliedCurrent = new double[]{leftMotor.getStatorCurrent().getValueAsDouble(), rightMotor.getStatorCurrent().getValueAsDouble()};

        inputs.anglerAppliedVolts = voltageAngle();
        inputs.anglerAppliedCurrent = angleMotor.getStatorCurrent().getValueAsDouble();

        inputs.feederAppliedVolts = feederMotor.getSupplyVoltage().getValueAsDouble();
        inputs.feederAppliedCurrent = feederMotor.getStatorCurrent().getValueAsDouble();

        inputs.beamBreakTriggered = beamBreak();
    }

    //Radians
    public double angleRad(){
        return encoder.getAbsolutePosition().getValueAsDouble() - ShooterConstants.kEncoderOffset; 
    }
    public double angleRadPerSec(){
        double velocity = (angleRad()-previousAngle)/(Timer.getFPGATimestamp() - previousTime);
        previousAngle = angleRad();
        previousTime = Timer.getFPGATimestamp();
        return velocity;
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

    public boolean[] speedCompensatedBounds(){
        double projection = angleRad() + angleRadPerSec() * 0.1;
        return new boolean[]{projection < ShooterConstants.kLowerBound, projection > ShooterConstants.kHigherBound};
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
        boolean[] boundsTriggered = speedCompensatedBounds();
        if(boundsTriggered[0] && voltage < 0){
          voltage = 0;
        }
    
        if(boundsTriggered[1] && voltage > 0){
          voltage = 0;
        }
    
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
