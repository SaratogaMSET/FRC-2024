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
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants;
import frc.robot.Constants.ShooterFeederConstants;
  
public class ShooterIOReal implements ShooterIO{
    TalonFX leftMotor = new TalonFX(ShooterFlywheelConstants.kLeftMotorPort, Constants.canbus);
    TalonFX rightMotor = new TalonFX(ShooterFlywheelConstants.kRightMotorPort, Constants.canbus);
    TalonFX angleMotor = new TalonFX(ShooterPivotConstants.kMotorPort, Constants.canbus);

    CANcoder encoder = new CANcoder(ShooterPivotConstants.kEncoderPort, Constants.canbus);
    // DigitalInput beamBreak = new DigitalInput(ShooterFlywheelConstants.kBeamBreakPort);

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
        rightMotor.setInverted(false);
        leftMotor.setControl(new CoastOut());
        rightMotor.setControl(new CoastOut());
        
        angleMotor.setInverted(false);
        
        angleMotor.setControl(new StaticBrake());

    }
    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.shooterRPS = new double[]{leftMotor.getVelocity().getValueAsDouble(), rightMotor.getVelocity().getValueAsDouble()};


        inputs.pivotRad = 2 * Math.PI * (-encoder.getAbsolutePosition().getValueAsDouble() - ShooterPivotConstants.kEncoderOffset);
        inputs.pivotRadPerSec = angleMotor.getVelocity().getValueAsDouble() * 2 * Math.PI / ShooterPivotConstants.kMotorGearing;

        inputs.shooterAppliedVolts = new double[]{leftMotor.getMotorVoltage().getValueAsDouble(), rightMotor.getMotorVoltage().getValueAsDouble()};
        inputs.shooterAppliedCurrent = new double[]{leftMotor.getStatorCurrent().getValueAsDouble(), rightMotor.getStatorCurrent().getValueAsDouble()};

        inputs.pivotAppliedVolts = angleMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotAppliedCurrent = angleMotor.getStatorCurrent().getValueAsDouble();


        // inputs.feederAppliedVolts = feederMotor.getMotorVoltage().getValueAsDouble();
        // inputs.feederAppliedCurrent = feederMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void setShooterVoltage(double voltage){
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotVoltage(double voltage){
        angleMotor.setVoltage(voltage);
    }

    @Override
    public void setBeamBreak(boolean isTriggered){}
}
