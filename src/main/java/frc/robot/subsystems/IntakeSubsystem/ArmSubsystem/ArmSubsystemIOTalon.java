package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.IntakeSubsystem.Arm;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;

public class ArmSubsystemIOTalon implements ArmSubsystemIO {
    ArmState state;
    TalonFX shoulder;
    CANSparkMax wrist;
    CANcoder shoulderEncoder;
    CANcoder wristEncoder;

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

    PIDController controller = new PIDController(Arm.ControlsConstants.k_P, 0.0, Arm.ControlsConstants.k_D);

    @Override
    public void updateInputs(ArmSubsystemIOInputs inputs) {
        inputs.armState = state;
        inputs.wristDegrees = wristGetDegrees();
        inputs.shoulderDegrees = shoulderGetDegrees();
        inputs.elevatorHeight = 0.0; //TODO: Fix this once elevator is completed
    }

    public ArmSubsystemIOTalon() {
        state = ArmState.NEUTRAL;

        shoulder = new TalonFX(Arm.INTAKE_SHOULDER_MOTOR, "Placeholder");
        wrist = new CANSparkMax(Arm.INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        shoulderEncoder = new CANcoder(Arm.INTAKE_SHOULDER_ENCODER, "Placeholder");
        wristEncoder = new CANcoder(Arm.INTAKE_WRIST_ENCODER, "Placeholder");

        // Set motor idle modes
        shoulder.setNeutralMode(Arm.ARM_NEUTRAL_MODE);
        wrist.setIdleMode(IdleMode.kBrake);

        // Set motor output configs for configuring deadband
        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();
        intakeTalonOutputConfigs.DutyCycleNeutralDeadband = Arm.NEUTRAL_VOLTAGE; // TODO: Tune

        intakeTalonConfigs.Slot0.kP = 0.0;
        intakeTalonConfigs.Slot0.kI = 0.0;
        intakeTalonConfigs.Slot0.kD = 0.0;
        intakeTalonConfigs.Slot0.kV = 0.0;
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0;// change later
        intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);

        shoulder.getConfigurator().apply(intakeTalonConfigs);
        wrist.setSmartCurrentLimit(Arm.INTAKE_WRIST_CURRENT_LIMIT);

        // Configure CANCoders
        CANcoderConfiguration intakeCANcoderConfigs = new CANcoderConfiguration();
        intakeCANcoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        shoulderEncoder.getConfigurator().apply(intakeCANcoderConfigs);
        wristEncoder.getConfigurator().apply(intakeCANcoderConfigs);
    }

    @Override
    public double shoulderGetRadians() {
        double raw_angle = Math.PI * 2
                * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }

    @Override
    public double shoulderGetDegrees() {
        double raw_angle = 360 * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }

    @Override
    public double wristGetRadians() {
        double raw_angle = Math.PI * 2
                * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Arm.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    @Override
    public double wristGetDegrees() {
        double raw_angle = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Arm.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    @Override
    public double shoulderGetCurrent() {
        return shoulder.getTorqueCurrent().getValueAsDouble();
    }

    @Override
    public double shoulderGetVoltage() {
        return shoulder.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void shoulderSetAngle(double angle, double velocity) {
        // Enforce bounds for velocity
        if (Math.abs(velocity) > 1)
            velocity = Math.signum(velocity);
        if (velocity < 0)
            velocity = 0;

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle
        if (angle > Arm.SHOULDER_HIGH_BOUND)
            angle = Arm.SHOULDER_HIGH_BOUND;
        if (angle < Arm.SHOULDER_LOW_BOUND)
            angle = Arm.SHOULDER_LOW_BOUND;

        // Calculate gravity ff + PID
        double error = (angle - shoulderGetDegrees()) / (Arm.SHOULDER_HIGH_BOUND - Arm.SHOULDER_LOW_BOUND);
        double gravity = Arm.ControlsConstants.k_G * Math.sin(shoulderGetRadians() + Arm.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

        // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
        if (angle > shoulderGetDegrees()) {
            shoulder.setVoltage((Arm.ControlsConstants.k_P * error * power) - gravity);
        } else {
            shoulder.setVoltage(((Arm.ControlsConstants.k_P * error) * power));
        }
    }

    @Override
    public void wristSetAngle(double angle, double velocity) { 
        // Enforce bounds for velocity
        if (Math.abs(velocity) > 1)
            velocity = Math.signum(velocity);
        if (velocity < 0)
            velocity = 0;

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle
        if (angle > Arm.WRIST_HIGH_BOUND)
            angle = Arm.WRIST_HIGH_BOUND;
        if (angle < Arm.WRIST_LOW_BOUND)
            angle = Arm.WRIST_LOW_BOUND;

        // Calculate gravity ff + PID
        double error = (angle - wristGetDegrees()) / (Arm.WRIST_HIGH_BOUND - Arm.WRIST_LOW_BOUND);
        double gravity = Arm.ControlsConstants.k_G * Math.sin(wristGetRadians() + Arm.WRIST_ENCODER_OFFSET_FROM_ZERO);

        // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
        if (angle > wristGetDegrees()) {
            wrist.setVoltage((Arm.ControlsConstants.k_P * error * power) - gravity);
        } else {
            wrist.setVoltage(((Arm.ControlsConstants.k_P * error) * power));
        }
    }

    @Override
    public void gravityCompensation() {
        shoulder.set(Arm.ControlsConstants.k_G * Math.cos(wristGetRadians() + Arm.WRIST_ENCODER_OFFSET_FROM_ZERO));
    }
}