package frc.robot.subsystems.Shooter;

import org.checkerframework.checker.units.qual.mol;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
  
public class ShooterIOReal implements ShooterIO{
    TalonFX leftMotor = new TalonFX(ShooterFlywheelConstants.kLeftMotorPort, Constants.CANBus);
    TalonFX rightMotor = new TalonFX(ShooterFlywheelConstants.kRightMotorPort, Constants.CANBus);
    TalonFX angleMotor = new TalonFX(ShooterPivotConstants.kMotorPort, Constants.CANBus);

    CANcoder encoder = new CANcoder(ShooterPivotConstants.kEncoderPort, Constants.CANBus);
    private VoltageOut leftVoltage = new VoltageOut(0.0).withEnableFOC(true);
    private VoltageOut rightVoltage = new VoltageOut(0.0).withEnableFOC(true);

    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);//.withEnableFOC(true);
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

        /** Angle Motor & Encoder Config */

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();

        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf; // IDK IF THIS WORKS
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; //TODO: iDK IF THIS WORKS. When turret moves down angle increases.
        cc_cfg.MagnetSensor.MagnetOffset = -0.124+(0.038 * 2); // TODO: FIND THIS VALUE. Highest Bound = 14 degrees. 0.038889 rotations. 
        encoder.getConfigurator().apply(cc_cfg);
        
        TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs angleCurrentLimitConfig = new CurrentLimitsConfigs();

        angleMotorConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        angleMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        angleMotorConfig.Feedback.RotorToSensorRatio = Constants.ShooterPivotConstants.kMotorGearing;

        /* Current Limits */

        angleCurrentLimitConfig.withStatorCurrentLimit(20);
        angleCurrentLimitConfig.withSupplyCurrentLimit(20); // Doesn't get called. 
        angleCurrentLimitConfig.withStatorCurrentLimitEnable(true);

        angleMotorConfig.withCurrentLimits(angleCurrentLimitConfig);

        angleMotorConfig.Slot0.kS = 0; //Tune first! 0.24
        angleMotorConfig.Slot0.kA = 0.0; //1.3
        angleMotorConfig.Slot0.kG = -0.25; //-1
        angleMotorConfig.Slot0.kV = 1; //Tune Second! 4
        angleMotorConfig.Slot0.kP = 50; //Tune Last! 8
        angleMotorConfig.Slot0.kI = 0;
        angleMotorConfig.Slot0.kD = 0;

        angleMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        
        MotionMagicConfigs motionMagicConfigs = angleMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 1000; //7
        motionMagicConfigs.MotionMagicAcceleration = 5000; //14
        motionMagicConfigs.MotionMagicJerk = 0; //30
        
        angleMotor.getConfigurator().apply(angleMotorConfig);
        angleMotor.setInverted(true);
        angleMotor.setNeutralMode(NeutralModeValue.Brake);
        angleMotor.setControl(new StaticBrake());
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.shooterRPS = new double[]{leftMotor.getVelocity().getValueAsDouble(), rightMotor.getVelocity().getValueAsDouble()};
        
        inputs.pivotRad = Units.rotationsToRadians(angleMotor.getPosition().getValueAsDouble());//2 * Math.PI * (-encoder.getAbsolutePosition().getValueAsDouble() - ShooterPivotConstants.kEncoderOffset);
        // angleMotor.setPosition(inputs.pivotRad / (2 * Math.PI) * ShooterPivotConstants.kMotorGearing, 0);
        inputs.pivotRadPerSec = Units.rotationsToRadians(angleMotor.getVelocity().getValueAsDouble());//angleMotor.getVelocity().getValueAsDouble() * 2 * Math.PI / ShooterPivotConstants.kMotorGearing;

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
    public void setPivotProfiled(double target, double additionalVoltage){

        Logger.recordOutput("RealOutputs/Shooter/Pivot/TargetRadsMotionMagic", target);
        target = Units.radiansToRotations(target);
        Logger.recordOutput("RealOutputs/Shooter/Pivot/TargetRotationMotionMagic", target);
        // Logger.recordOutput("RealOutputs/Shooter/Pivot/Rotations");
        angleMotor.setControl(motionMagicVoltage.withPosition(target).withFeedForward(0));
    }
    @Override
    public void setPivotVoltage(double voltage){
        angleMotor.setVoltage(voltage);
    }

    @Override
    public void setBeamBreak(boolean isTriggered){}
}