package frc.robot.subsystems.Turret;


import org.checkerframework.checker.units.qual.A;
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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class TurretIOReal implements TurretIO{

    TalonFX m_motor = new TalonFX(Constants.TurretConstants.kMotorPort, Constants.CANBus);
    CANcoder encoder = new CANcoder(Constants.TurretConstants.kEncoderPort, Constants.CANBus);
    double voltage = 0;

    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    public TurretIOReal() {
        configureCANCoder();
        configureTalonFX();
    }

    private void configureCANCoder() {
        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();

        // TODO: Set sensor properties based on CANcoder requirements
        // Example:
        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        cc_cfg.MagnetSensor.MagnetOffset = -.6;

        encoder.getConfigurator().apply(cc_cfg);
    }

    private void configureTalonFX() {
        TalonFXConfiguration turretTalonConfigs = new TalonFXConfiguration();

        // Current Limits
        turretTalonConfigs.CurrentLimits.StatorCurrentLimit = 20; // Change later if needed
        turretTalonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        // Feedback Setup (using CANcoder)
        turretTalonConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        turretTalonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        // Sensor-to-Mechanism Ratio
        turretTalonConfigs.Feedback.SensorToMechanismRatio = 1.0; 
        turretTalonConfigs.Feedback.RotorToSensorRatio = TurretConstants.kMotorGearing;

        // Slot 0 Configs (for future control loops)
        var slot0Configs = turretTalonConfigs.Slot0;

        // **PIDF Gains (commented out, set based on your control needs)**
        slot0Configs.kS = 0.45;  // Feedforward gain for static friction
        slot0Configs.kA = 0;  // Feedforward gain for acceleration
        slot0Configs.kV = 1; //Units.degreesToRotations(TurretConstants.kV);  // 50 degrees / second per volt
        // Feedforward gain for velocity  // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kP = 0;  // Proportional gain
        slot0Configs.kI = 0;  // Integral gain
        slot0Configs.kD = 0.0;  // Derivative gain
        slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

        // **Motion Magic Configuration (commented out, use for planned motions)**
        MotionMagicConfigs motionMagicConfigs = turretTalonConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 3;  
        motionMagicConfigs.MotionMagicAcceleration = 6;  
        motionMagicConfigs.MotionMagicJerk = 18;       

        // Motor Output Configs
        MotorOutputConfigs turretTalonOutputConfigs = new MotorOutputConfigs();
        turretTalonOutputConfigs.PeakForwardDutyCycle = 1;
        turretTalonOutputConfigs.PeakReverseDutyCycle = 1;
        turretTalonOutputConfigs.DutyCycleNeutralDeadband = 0.0;
        turretTalonOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        // Closed-Loop Ramps
        ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
        voltageRampConfig.VoltageClosedLoopRampPeriod = 0;

        turretTalonConfigs.withMotorOutput(turretTalonOutputConfigs);
        turretTalonConfigs.withClosedLoopRamps(voltageRampConfig);
        turretTalonConfigs.withCurrentLimits(turretTalonConfigs.CurrentLimits);

        m_motor.getConfigurator().apply(turretTalonConfigs);
        m_motor.setInverted(true);
        m_motor.setNeutralMode(NeutralModeValue.Brake);
        m_motor.setControl(new StaticBrake());
    }

    public void updateInputs(TurretIOInputs inputs){
        inputs.turretRad = Units.rotationsToRadians(m_motor.getPosition().getValueAsDouble());//2 * Math.PI * (encoder.getAbsolutePosition().getValueAsDouble() - Constants.TurretConstants.kEncoderOffset);
        inputs.turretRadPerSec = Units.rotationsToRadians(m_motor.getVelocity().getValueAsDouble()); // * 2 * Math.PI / TurretConstants.kMotorGearing;
        inputs.turretVoltage = m_motor.getMotorVoltage().getValueAsDouble();
        inputs.turretCurrent = m_motor.getStatorCurrent().getValueAsDouble();
    }
    @Override
    public void setProfiled(double target, double additionalVoltage){
        // Target in radians. 
        Logger.recordOutput("RealOutputs/Turret/TargetRadiansMotionMagic", target);
        target = Units.radiansToRotations(target);
        
        // MotionMagicVoltage control = new MotionMagicVoltage(target, true, additionalVoltage, 0, false, false, false);
        Logger.recordOutput("RealOutputs/Turret/TargetRotationMotionMagic", target);
        // Logger.recordOutput("RealOutputs/Intake/Shoulder/MotionMagicFF", FF);
        m_motor.setControl(motionMagicVoltage.withPosition(target).withFeedForward(additionalVoltage));
    }
    @Override
    public void setVoltage(double voltage){
        m_motor.setVoltage(voltage);
    }
}
