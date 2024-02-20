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
        inputs.shooterRPM = shooterRPM;
        inputs.theta = theta;
        inputs.thetaPerSeconds = thetaPerSeconds;

        inputs.shooterAppliedVolts = new double[]{shooterVoltage, shooterVoltage};
        inputs.shooterAppliedCurrent = new double[]{shooterSim.getCurrentDrawAmps()};

        inputs.anglerAppliedVolts = anglerVoltage;
        inputs.anglerAppliedCurrent = anglerSim.getCurrentDrawAmps();

        inputs.feederAppliedVolts = feederVoltage;
        inputs.feederAppliedCurrent = feederSim.getCurrentDrawAmps();

        inputs.beamBreakTriggered = beamBreakTriggered;
    }

    @Override
    public void setDesiredAngler(double radians, double radiansPerSecond){}
    @Override

    public void setDesiredRPM(double RPM){}
    @Override

    public void setShooterVoltage(double voltage){}
    @Override

    public void setAnglerVoltage(double voltage){}
    @Override

    public void setFeederVoltage(double voltage){}

    @Override
    public void resetThetaEncoder(){}

    @Override
    public void setBeamBreak(boolean isTriggered){}
}
