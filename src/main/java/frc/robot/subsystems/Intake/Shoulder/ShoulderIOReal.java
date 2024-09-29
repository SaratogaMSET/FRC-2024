package frc.robot.subsystems.Intake.Shoulder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Intake.Shoulder;
import org.littletonrobotics.junction.Logger;

public class ShoulderIOReal implements ShoulderIO {

  TalonFX motor = new TalonFX(Shoulder.MOTOR, Constants.CANBus);
  CANcoder encoder = new CANcoder(Shoulder.ENCODER, Constants.CANBus);

  double previousError = 0; // Move to constants, preferably in nested class within Arm class
  double errorDT;
  MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);

  public ShoulderIOReal() {

    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();

    cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = -0.257 - 0.024 + 0.05 - 0.02; // TODO: FIND THIS VALUE
    // //Units.radiansToRotations(Intake.Shoulder.ENCODER_OFFSET_FROM_ZERO);
    encoder.getConfigurator().apply(cc_cfg);

    TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();
    intakeTalonConfigs.CurrentLimits.StatorCurrentLimit = 30; // change later
    intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    intakeTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeTalonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    intakeTalonConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();

    intakeTalonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    intakeTalonConfigs.Feedback.SensorToMechanismRatio = 1.0; // Cancoder to shoulder rotation ratio

    // Ratio of one motor rotor rotation to 1 rotation of shoulder movement.
    intakeTalonConfigs.Feedback.RotorToSensorRatio = Shoulder.GEAR_RATIO;

    intakeTalonConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    var slot0Configs = intakeTalonConfigs.Slot0;
    slot0Configs.kG = Shoulder.k_G;
    slot0Configs.kS = 0; // Add 0.0 V output to overcome static friction
    slot0Configs.kV = 4; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output. It's radians
    slot0Configs.kP = 40; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0.0; // no output for integrated error
    slot0Configs.kD = 3; // no output for error derivative

    MotionMagicConfigs motionMagicConfigs = intakeTalonConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 5;
    motionMagicConfigs.MotionMagicAcceleration = 10;
    motionMagicConfigs.MotionMagicJerk = 40;

    intakeTalonConfigs.CurrentLimits.StatorCurrentLimit = 30; // change later
    intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    intakeTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeTalonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

    MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();

    intakeTalonOutputConfigs.DutyCycleNeutralDeadband = 0.0;
    intakeTalonOutputConfigs.NeutralMode = NeutralModeValue.Brake; // change back to brake?

    intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);
    motor.getConfigurator().apply(intakeTalonConfigs);
    motor.setInverted(true);
    motor.setControl(new StaticBrake()); // change back to StaticBrake()?
  }

  @Override
  /** Updates inputs for shoulder voltage, current and angle */
  public void updateInputs(ShoulderIOInputs inputs) {
    inputs.shoulderVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.shoulderCurrent = motor.getTorqueCurrent().getValueAsDouble();
    inputs.shoulderRads =
        Units.rotationsToRadians(
            motor
                .getPosition()
                .getValueAsDouble()); // encoder.getAbsolutePosition().getValueAsDouble(); // -2 *
    // Math.PI * (encoder.getAbsolutePosition().getValueAsDouble()
    // - Shoulder.ENCODER_OFFSET);
    inputs.shoulderRadPerSecs =
        Units.rotationsToRadians(
            motor
                .getVelocity()
                .getValueAsDouble()); // -2 * Math.PI * motor.getVelocity().getValueAsDouble() /
    // Shoulder.GEAR_RATIO;
    inputs.motorShoulderRads =
        Units.rotationsToRadians(
            motor.getRotorPosition().getValueAsDouble()); // inputs.shoulderRads;
    inputs.motorRadPerSecs = Units.rotationsToRadians(motor.getRotorVelocity().getValueAsDouble());
  }

  @Override
  /** Sets shoulder voltage */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  /** Sets shoulder with motion magic. Target is angle in radians. */
  public void setProfiled(double target, double FF) {

    /* Tranform to rotations*/
    target = Units.radiansToRotations(target);

    Logger.recordOutput("RealOutputs/Intake/Shoulder/TargetRotationMotionMagic", target);
    Logger.recordOutput("RealOutputs/Intake/Shoulder/MotionMagicFF", FF);
    // MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(target);
    motor.setControl(motionMagicVoltage.withPosition(target).withFeedForward(FF));
  }
}
