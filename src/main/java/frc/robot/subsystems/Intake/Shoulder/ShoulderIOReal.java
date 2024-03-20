package frc.robot.subsystems.Intake.Shoulder;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.Shoulder;

public class ShoulderIOReal implements ShoulderIO {

    TalonFX motor = new TalonFX(Shoulder.MOTOR, Constants.CANBus);
    CANcoder encoder = new CANcoder(Shoulder.ENCODER,Constants.CANBus);

    // public final double motor_encoder_to_mechanism_ratio = -2 * Math.PI * motor.getPosition().getValueAsDouble() / Shoulder.GEAR_RATIO;

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

    public ShoulderIOReal(){

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();

        cc_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive; //TODO: iDK IF THIS WORKS.
        cc_cfg.MagnetSensor.MagnetOffset = 0.0; // TODO: FIND THIS VALUE //Units.radiansToRotations(Intake.Shoulder.ENCODER_OFFSET_FROM_ZERO);
        encoder.getConfigurator().apply(cc_cfg);

        
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();
        intakeTalonConfigs.CurrentLimits.StatorCurrentLimit = 30;// change later
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeTalonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeTalonConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();

        intakeTalonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        intakeTalonConfigs.Feedback.SensorToMechanismRatio = 1.0;

        // Ratio of one sensor/encoder rotation to 1 rotation of shoulder movement.
        intakeTalonConfigs.Feedback.RotorToSensorRatio = Shoulder.GEAR_RATIO;

        var slot0Configs = intakeTalonConfigs.Slot0;
        slot0Configs.kS = 0.0; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.0; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.0; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = Constants.Intake.Shoulder.k_P; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0.0; // no output for integrated error
        slot0Configs.kD = Constants.Intake.Shoulder.k_D; // no output for error derivative

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.2;
        motionMagicConfigs.MotionMagicAcceleration = 0.4;
        motionMagicConfigs.MotionMagicJerk = 0.6;

        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        intakeTalonOutputConfigs.DutyCycleNeutralDeadband = 0.0;
        intakeTalonOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);

        motor.getConfigurator().apply(intakeTalonConfigs);
        motor.setInverted(true);
    }
    @Override
     /**Updates inputs for shoulder voltage, current and angle*/
    public void updateInputs(ShoulderIOInputs inputs) {
        inputs.shoulderVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.shoulderCurrent = motor.getTorqueCurrent().getValueAsDouble();
        inputs.shoulderRads = encoder.getAbsolutePosition().getValueAsDouble(); // -2 * Math.PI * (encoder.getAbsolutePosition().getValueAsDouble() - Shoulder.ENCODER_OFFSET);
        inputs.shoulderRadPerSecs = -2 * Math.PI * motor.getVelocity().getValueAsDouble() / Shoulder.GEAR_RATIO;
        inputs.motorShoulderRads = inputs.shoulderRads;
        inputs.motorRadPerSecs = motor.getVelocity().getValueAsDouble();
    }

    @Override
    /**Sets shoulder voltage*/
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    /**Sets shoulder with motion magic*/
    public void setProfiled(double target, double FF) {
        MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(target);
        motionMagicVoltage.withFeedForward(FF);
        motor.setControl(motionMagicVoltage);
    }

}
