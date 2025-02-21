package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import org.littletonrobotics.junction.Logger;

public class TurretIOReal implements TurretIO {

  TalonFX m_motor = new TalonFX(Constants.TurretConstants.kMotorPort, Constants.CANBus);
  CANcoder encoder = new CANcoder(Constants.TurretConstants.kEncoderPort, Constants.CANBus);
  double voltage = 0;

  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);

  public TurretIOReal() {
    configureCANCoder();
    configureTalonFX();
  }

  private void configureCANCoder() {
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();

    // TODO: Set sensor properties based on CANcoder requirements
    cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = -.6;

    encoder.getConfigurator().apply(cc_cfg);
  }

  private void configureTalonFX() {
    TalonFXConfiguration turretTalonConfigs = new TalonFXConfiguration();

    // Current Limits
    turretTalonConfigs.CurrentLimits.StatorCurrentLimit = 20; // Change later if needed
    turretTalonConfigs.CurrentLimits.SupplyCurrentLimit = 20;
    turretTalonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    turretTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;

    // Feedback Setup (using CANcoder)
    turretTalonConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    turretTalonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    // Sensor-to-Mechanism Ratio
    turretTalonConfigs.Feedback.SensorToMechanismRatio = 1.0;
    turretTalonConfigs.Feedback.RotorToSensorRatio = TurretConstants.kMotorGearing;

    // Slot 0 Configs (for future control loops)
    var slot0Configs = turretTalonConfigs.Slot0;

    // **PIDF Gains (commented out, set based on your control needs)**
    slot0Configs.kS = 0.2; // Feedforward gain for static friction
    slot0Configs.kA = 0.01; // Feedforward gain for acceleration
    slot0Configs.kV =
        5; // Feedforward gain for velocity  // A velocity target of 1 rps results in 5 V output
    slot0Configs.kP = 30; // Proportional gain
    slot0Configs.kI = 0; // Integral gain
    slot0Configs.kD = 0.0; // Derivative gain
    slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    // **Motion Magic Configuration (commented out, use for planned motions)** :p
    MotionMagicConfigs motionMagicConfigs = turretTalonConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 2000;
    motionMagicConfigs.MotionMagicAcceleration = 5000;
    motionMagicConfigs.MotionMagicJerk = 0; // 4000;

    // Motor Output Configs
    MotorOutputConfigs turretTalonOutputConfigs = new MotorOutputConfigs();
    turretTalonOutputConfigs.PeakForwardDutyCycle = 1;
    turretTalonOutputConfigs.PeakReverseDutyCycle = 1;
    turretTalonOutputConfigs.DutyCycleNeutralDeadband = 0.0;
    turretTalonOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    // Closed-Loop Ramps
    ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
    voltageRampConfig.VoltageClosedLoopRampPeriod = 0;

    var limitConfigs = new HardwareLimitSwitchConfigs();
    limitConfigs.ReverseLimitEnable = false;
    limitConfigs.ForwardLimitEnable = false;

    turretTalonConfigs.withMotorOutput(turretTalonOutputConfigs);
    turretTalonConfigs.withClosedLoopRamps(voltageRampConfig);
    turretTalonConfigs.withCurrentLimits(turretTalonConfigs.CurrentLimits);
    turretTalonConfigs.withHardwareLimitSwitch(limitConfigs);

    m_motor.getConfigurator().apply(turretTalonConfigs);
    m_motor.setInverted(true);
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    m_motor.setControl(new StaticBrake());
  }

  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretRad = Units.rotationsToRadians(m_motor.getPosition().getValueAsDouble());
    inputs.turretRadPerSec = Units.rotationsToRadians(m_motor.getVelocity().getValueAsDouble());
    inputs.turretVoltage = m_motor.getMotorVoltage().getValueAsDouble();
    inputs.turretCurrent = m_motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  /** Sets the Motion Magic target to target and applies additionalVoltage volts of feedforward */
  public void setProfiled(double target, double additionalVoltage) {
    // Target in radians.
    Logger.recordOutput("RealOutputs/Turret/TargetRadiansMotionMagic", target);
    target = Units.radiansToRotations(target);
    Logger.recordOutput("RealOutputs/Turret/TargetRotationMotionMagic", target);
    m_motor.setControl(motionMagicVoltage.withPosition(target).withFeedForward(additionalVoltage));
  }

  @Override
  public void setVoltage(double voltage) {
    m_motor.setVoltage(voltage);
  }
}
