package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ShoulderSubsystem;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;

public class ShoulderSubsystemIOTalonFX implements ShoulderSubsystemIO{

    ArmState state;
    TalonFX shoulder;
    CANcoder shoulderEncoder;

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;

    PIDController controller = new PIDController(Arm.ControlsConstants.k_P, 0.0, Arm.ControlsConstants.k_D);

    public ShoulderSubsystemIOTalonFX(){
        state = ArmState.NEUTRAL;

        shoulder = new TalonFX(Arm.INTAKE_SHOULDER_MOTOR);
        shoulder.setNeutralMode(Arm.ARM_NEUTRAL_MODE);

        shoulderEncoder = new CANcoder(Arm.INTAKE_SHOULDER_ENCODER, "Placeholder");


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

        // Configure CANCoders
        CANcoderConfiguration intakeCANcoderConfigs = new CANcoderConfiguration();
        intakeCANcoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        shoulderEncoder.getConfigurator().apply(intakeCANcoderConfigs);
    }

    @Override
    public void updateInputs(ShoulderSubsystemIOInputs inputs){
        inputs.shoulderDegrees = getShoulderDegrees();
        inputs.shoulderCurrent = getShoulderCurrent();
        inputs.shoulderVoltage = getShoulderVoltage();
    }

    @Override
    public void setAngle(double angle, double speed) {
        double staticVoltage = 0.0;
        double directionOfTravel = Math.signum(angle - getShoulderDegrees());
        // Enforce bounds for velocity
        if (speed > Arm.MAX_SHOULDER_SPEED)
            speed = directionOfTravel * Arm.MAX_SHOULDER_SPEED;
        if (speed < 0)
            speed = 0;
        if (speed < Arm.STATIC_SPEED)
            staticVoltage = Arm.SHOULDER_OVERCOME_STATIC_VOLTAGE * directionOfTravel;

        // Calculate the voltage draw 
        double power = 12 * speed * directionOfTravel;

        // Enforce bounds on angle
        if (angle > Arm.SHOULDER_HIGH_BOUND)
            angle = Arm.SHOULDER_HIGH_BOUND;
        if (angle < Arm.SHOULDER_LOW_BOUND)
            angle = Arm.SHOULDER_LOW_BOUND;

        // Calculate gravity ff + PID
        double error = (angle - getShoulderDegrees()) / (Arm.SHOULDER_HIGH_BOUND - Arm.SHOULDER_LOW_BOUND);
        double gravity = Arm.ControlsConstants.k_G * Math.sin(getShoulderRadians() + Arm.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

        // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
        if (angle > getShoulderDegrees()) {
            shoulder.setVoltage((Arm.ControlsConstants.k_P * error * power) - gravity + staticVoltage);
        } else {
            shoulder.setVoltage(((Arm.ControlsConstants.k_P * error) * power) + staticVoltage);
        }

        SmartDashboard.putNumber("Shoulder Power: ", power);
        SmartDashboard.putNumber("Shoulder Error: ", error);
    }

    @Override
    public void setVoltage(double voltage){
        shoulder.setVoltage(voltage);
    }

    public double getShoulderDegrees() {
        double raw_angle = 360 * (shoulder.getPosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
        //double raw_angle = 360 * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }

    public double getShoulderCurrent() {
        return shoulder.getTorqueCurrent().getValueAsDouble();
    }

    public double getShoulderVoltage() {
        return shoulder.getMotorVoltage().getValueAsDouble();
    }

    public double getShoulderRadians() {
        double raw_angle = Math.PI * 2
                * (shoulder.getPosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
                // double raw_angle = Math.PI * 2
                // * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }


}
