package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class WristSubsystemIOTalonFX implements WristSubsystemIO {
    ArmState state;
    TalonFX wrist;
    CANcoder wristEncoder;

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

    PIDController controller = new PIDController(Arm.ControlsConstants.k_P, 0.0, Arm.ControlsConstants.k_D);

    public WristSubsystemIOTalonFX(){
        state = ArmState.NEUTRAL;

        wrist = new TalonFX(Arm.INTAKE_WRIST_MOTOR);
        wrist.setNeutralMode(Arm.ARM_NEUTRAL_MODE);

        wristEncoder = new CANcoder(Arm.INTAKE_WRIST_ENCODER, "Placeholder");


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

        wrist.getConfigurator().apply(intakeTalonConfigs);

        // Configure CANCoders
        CANcoderConfiguration intakeCANcoderConfigs = new CANcoderConfiguration();
        intakeCANcoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        wristEncoder.getConfigurator().apply(intakeCANcoderConfigs);
    }

    @Override
    public void updateInputs(WristSubsystemIOInputs inputs){
        inputs.wristDegrees = getWristDegrees();
        inputs.wristCurrent = getWristCurrent();
        inputs.wristVoltage = getWristVoltage();
    }

    @Override
    public void setAngle(double angle, double speed) {
        double staticVoltage = 0.0;
        double directionOfTravel = Math.signum(angle - getWristDegrees());
        // Enforce bounds for velocity
        if (speed > Arm.MAX_WRIST_SPEED)
            speed = directionOfTravel * Arm.MAX_WRIST_SPEED;
        if (speed < 0)
            speed = 0;
        if (speed < Arm.STATIC_SPEED)
            staticVoltage = Arm.WRIST_OVERCOME_STATIC_VOLTAGE * directionOfTravel;

        // Calculate the voltage draw 
        double power = 12 * speed * directionOfTravel;

        // Enforce bounds on angle
        if (angle > Arm.WRIST_HIGH_BOUND)
            angle = Arm.WRIST_HIGH_BOUND;
        if (angle < Arm.WRIST_LOW_BOUND)
            angle = Arm.WRIST_LOW_BOUND;

        // Calculate gravity ff + PID
        double error = (angle - getWristDegrees()) / (Arm.WRIST_HIGH_BOUND - Arm.WRIST_LOW_BOUND);
        double gravity = Arm.ControlsConstants.k_G * Math.sin(getWristRadians() + Arm.WRIST_ENCODER_OFFSET_FROM_ZERO);

        // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
        if (angle > getWristDegrees()) {
            wrist.setVoltage((Arm.ControlsConstants.k_P * error * power) - gravity + staticVoltage);
        } else {
            wrist.setVoltage(((Arm.ControlsConstants.k_P * error) * power) + staticVoltage);
        }

        SmartDashboard.putNumber("Wrist Power: ", power);
        SmartDashboard.putNumber("Wrist Error: ", error);
    }

    @Override
    public void setVoltage(double voltage){
        wrist.setVoltage(voltage);
    }

    public double getWristDegrees() {
        double raw_angle = 360 * (wrist.getPosition().getValueAsDouble() - Arm.WRIST_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }

    public double getWristCurrent() {
        return wrist.getTorqueCurrent().getValueAsDouble();
    }

    public double getWristVoltage() {
        return wrist.getMotorVoltage().getValueAsDouble();
    }

    public double getWristRadians() {
        double raw_angle = Math.PI * 2
                * (wrist.getPosition().getValueAsDouble() - Arm.WRIST_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
                // double raw_angle = Math.PI * 2
        return raw_angle;
    }

}