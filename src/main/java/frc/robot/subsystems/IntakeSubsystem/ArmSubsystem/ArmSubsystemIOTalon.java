package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystem.Arm;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import frc.robot.Constants.IntakeSubsystem.Arm.GroundNeutralPerimeterConstants;
import frc.robot.Constants.IntakeSubsystem.Arm.AmpScoringPositions;
import frc.robot.Constants.IntakeSubsystem.Arm.SourceScoringPositions;
import frc.robot.Constants.IntakeSubsystem.Roller;

public class ArmSubsystemIOTalon implements ArmSubsystemIO {

    ArmState state;
    TalonFX shoulder;
    CANSparkMax wrist; // TODO: Needs to be a NEO
    CANcoder shoulderEncoder;
    CANcoder wristEncoder;

    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    public double k_G = 0;
    double k_P = 0;
    double k_D = 0.000;
    double k_I = 0.000;
    double errorDT;
    double prevError;

    PIDController controller = new PIDController(k_P, 0.0, k_D);

    @Override
    public ArmSubsystemIOInputsAutoLogged updateInputs() {
        var inputs = new ArmSubsystemIOInputsAutoLogged();
        // inputs.moduleNumber = moduleNumber;

        // inputs.steerTemparature = steerMotor.getTemperature();
        return inputs;
    }

    /*
     * TODO: Update for new Phoenix version
     * SupplyCurrentLimitConfiguration ArmLimit = new
     * SupplyCurrentLimitConfiguration(
     * true,
     * Constants.Drivetrain.driveContinuousCurrentLimit,
     * GroundIntake.currentLimit,
     * Constants.Drivetrain.drivePeakCurrentDuration);
     */

    public ArmSubsystemIOTalon() {
        state = ArmState.NEUTRAL;

        shoulder = new TalonFX(Arm.INTAKE_SHOULDER_MOTOR, "Placeholder");
        wrist = new CANSparkMax(Arm.INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        shoulderEncoder = new CANcoder(Arm.INTAKE_SHOULDER_ENCODER, "Placeholder");
        wristEncoder = new CANcoder(Arm.INTAKE_WRIST_ENCODER, "Placeholder");

        shoulder.setNeutralMode(Arm.ARM_NEUTRAL_MODE);
        wrist.setIdleMode(IdleMode.kBrake);
        // Set motor output configs for configuring deadband
        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();
        intakeTalonOutputConfigs.DutyCycleNeutralDeadband = Arm.NEUTRAL_VOLTAGE; // TODO: Tune
                                                                                     // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode

        intakeTalonConfigs.Slot0.kP = 0.0;
        intakeTalonConfigs.Slot0.kI = 0.0;
        intakeTalonConfigs.Slot0.kD = 0.0;
        intakeTalonConfigs.Slot0.kV = 0.0;
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0;// change later
        intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);

        shoulder.getConfigurator().apply(intakeTalonConfigs);

        CANcoderConfiguration intakeCANcoderConfigs = new CANcoderConfiguration();
        intakeCANcoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        shoulderEncoder.getConfigurator().apply(intakeCANcoderConfigs);
        wristEncoder.getConfigurator().apply(intakeCANcoderConfigs);
    }

    @Override
    public double shoulderGetRadians() {
        // angle ret 0-1
        double raw_angle = Math.PI * 2
                * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming
                                                                                                          // encoder
                                                                                                          // offset is
                                                                                                          // in native
                                                                                                          // units
                                                                                                          // (rotations
                                                                                                          // [0, 1))
        return raw_angle;
    }

    @Override
    public double shoulderGetDegrees() {
        // angle ret 0-1 //wait it doesnt u gotta scale yup
        double raw_angle = 360 * (shoulderEncoder.getAbsolutePosition().getValueAsDouble() - Arm.SHOULDER_ENCODER_OFFSET); // Assuming
                                                                                                                         // encoder
                                                                                                                         // offset
                                                                                                                         // is
                                                                                                                         // in
                                                                                                                         // native
                                                                                                                         // units
                                                                                                                         // (rotations
                                                                                                                         // [0,
                                                                                                                         // 1))
        return raw_angle;
    }

    @Override
    public double wristGetRadians() {
        // angle ret 0-1
        double raw_angle = Math.PI * 2
                * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Arm.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    @Override
    public double wristGetDegrees() {
        // angle ret 0-1
        double raw_angle = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Arm.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    @Override
    public double shoulderGetCurrent() {
        return shoulder.getTorqueCurrent().getValueAsDouble();
    }
    // public double wristGetCurrent(){
    // return wrist.get
    // }

    // ret OUTPUT voltage
    public double shoulderGetVoltage() {
        return shoulder.getMotorVoltage().getValueAsDouble();
    }

    // ret OUTPUT voltage
    // public double wristGetVoltage(){
    // return wrist.getMotorVoltage().getValueAsDouble();
    // }

    @Override
    public void shoulderSetAngle(double angle, double powerPercent) { // Assuming powerPercent is not signed (because it's
                                                                   // a user input)
        if (powerPercent > 100)
            powerPercent = 100;
        if (powerPercent < 0)
            powerPercent = 0;

        double power = 12 * powerPercent / 100;
        if (angle > Arm.SHOULDER_HIGH_BOUND)
            angle = Arm.SHOULDER_HIGH_BOUND;
        if (angle < Arm.SHOULDER_LOW_BOUND)
            angle = Arm.SHOULDER_LOW_BOUND;
        double error = (angle - shoulderGetDegrees()) / (Arm.SHOULDER_HIGH_BOUND - Arm.SHOULDER_LOW_BOUND);
        double gravity = k_G * Math.sin(shoulderGetRadians() + Arm.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

        if (angle > shoulderGetDegrees()) {
            shoulder.setVoltage((k_P * error * power) - gravity);
        } else {
            shoulder.setVoltage(((k_P * error) * power));
        }
        // SmartDashboard.putNumber("Shoulder error", (k_P * error * power));
    }

    @Override
    public void wristSetAngle(double angle, double powerPercent) { // Assuming powerPercent is not signed (because it's
                                                                   // a user input)
        if (powerPercent > 100)
            powerPercent = 100;
        if (powerPercent < 0)
            powerPercent = 0;

        double power = 12 * powerPercent / 100;
        if (angle > Arm.WRIST_HIGH_BOUND)
            angle = Arm.WRIST_HIGH_BOUND;
        if (angle < Arm.WRIST_LOW_BOUND)
            angle = Arm.WRIST_LOW_BOUND;
        double error = (angle - wristGetDegrees()) / (Arm.WRIST_HIGH_BOUND - Arm.WRIST_LOW_BOUND);
        double gravity = k_G * Math.sin(wristGetRadians() + Arm.WRIST_ENCODER_OFFSET_FROM_ZERO);

        if (angle > wristGetDegrees()) {
            wrist.setVoltage((k_P * error * power) - gravity);
        } else {
            wrist.setVoltage(((k_P * error) * power));
        }
        SmartDashboard.putNumber("Shoulder error", (k_P * error * power));
    }

    @Override
    public void gravityCompensation() {
        shoulder.set(k_G * Math.cos(wristGetRadians() + Arm.WRIST_ENCODER_OFFSET_FROM_ZERO));
    }

    @Override
    public ArmState getArmState() {
        return state;
    }

    @Override
    public void setArmState(ArmState state) {
        this.state = state;
    }
}