package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants;
import frc.robot.Constants.ShooterFeederConstants;
  
public class ShooterIOReal implements ShooterIO{
    TalonFX leftMotor = new TalonFX(ShooterFlywheelConstants.kLeftMotorPort, Constants.CANBus);
    TalonFX rightMotor = new TalonFX(ShooterFlywheelConstants.kRightMotorPort, Constants.CANBus);
    TalonFX angleMotor = new TalonFX(ShooterPivotConstants.kMotorPort, Constants.CANBus);

    CANcoder encoder = new CANcoder(ShooterPivotConstants.kEncoderPort, Constants.CANBus);
    private VoltageOut leftVoltage = new VoltageOut(0.0).withEnableFOC(true);
    private VoltageOut rightVoltage = new VoltageOut(0.0).withEnableFOC(true);
    // DigitalInput beamBreak = new DigitalInput(ShooterFlywheelConstants.kBeamBreakPort);

    public ShooterIOReal(){
        TalonFXConfiguration generalConfig = new TalonFXConfiguration();
        MotorOutputConfigs motorConfig = new MotorOutputConfigs();
        ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
        CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

        motorConfig.withPeakForwardDutyCycle(1);
        motorConfig.withPeakReverseDutyCycle(1);
        motorConfig.DutyCycleNeutralDeadband = 0.0;
        
        voltageRampConfig.withVoltageClosedLoopRampPeriod(0.5);

        currentLimitConfig.withStatorCurrentLimit(40); //was 60
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
        
        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs angleCurrentLimitConfig = new CurrentLimitsConfigs();

        angleCurrentLimitConfig.withStatorCurrentLimit(20);
        angleCurrentLimitConfig.withSupplyCurrentLimit(20);
        angleCurrentLimitConfig.withStatorCurrentLimitEnable(true);

        angleMotorConfig.withCurrentLimits(angleCurrentLimitConfig);
        angleMotor.getConfigurator().apply(angleMotorConfig);
        angleMotor.setInverted(true);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
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
    }

    @Override
    public void setShooterVoltage(double voltage){
        leftMotor.setControl(leftVoltage.withOutput(voltage));
        rightMotor.setControl(rightVoltage.withOutput(voltage));
    }

    @Override
    public void setPivotVoltage(double voltage){
        angleMotor.setVoltage(voltage);
    }

    @Override
    public void setBeamBreak(boolean isTriggered){}
}