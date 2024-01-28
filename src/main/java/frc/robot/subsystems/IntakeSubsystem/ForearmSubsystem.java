package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystem.Forearm;
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
import frc.robot.Constants.IntakeSubsystem.Forearm.ArmState;

public class ForearmSubsystem extends SubsystemBase{
    ArmState state;
    TalonFX elbow; 
    CANSparkMax wrist;    // TODO: Needs to be a NEO
    CANcoder elbowEncoder;
    CANcoder wristEncoder;

    double previousError = 0;   // Move to constants, preferably in nested class within Forearm class
    public double k_G = 0; 
    double k_P = 0; 
    double k_D = 0.000;
    double k_I = 0.000;
    double errorDT;
    double prevError;

    PIDController controller = new PIDController(k_P, 0.0 ,k_D);

        /* TODO: Update for new Phoenix version
    SupplyCurrentLimitConfiguration ForearmLimit = new SupplyCurrentLimitConfiguration(
            true, 
            Constants.Drivetrain.driveContinuousCurrentLimit, 
            GroundIntake.currentLimit, 
            Constants.Drivetrain.drivePeakCurrentDuration);*/

    public ForearmSubsystem(){
        state = ArmState.NEUTRAL;

        elbow = new TalonFX(Forearm.INTAKE_ELBOW_MOTOR, "Placeholder");
        wrist = new CANSparkMax(Forearm.INTAKE_WRIST_MOTOR, MotorType.kBrushless);
        elbowEncoder = new CANcoder(Forearm.INTAKE_ELBOW_ENCODER,  "Placeholder");
        wristEncoder = new CANcoder(Forearm.INTAKE_WRIST_ENCODER,  "Placeholder");

        elbow.setNeutralMode(Forearm.FOREARM_NEUTRAL_MODE);
        wrist.setIdleMode(IdleMode.kBrake);
        // Set motor output configs for configuring deadband
        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();
        intakeTalonOutputConfigs.DutyCycleNeutralDeadband = Forearm.NEUTRAL_VOLTAGE;    // TODO: Tune  https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode        

        intakeTalonConfigs.Slot0.kP = 0.0; 
        intakeTalonConfigs.Slot0.kI = 0.0;
        intakeTalonConfigs.Slot0.kD = 0.0;
        intakeTalonConfigs.Slot0.kV = 0.0;
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0;//change later
        intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);
        
        elbow.getConfigurator().apply(intakeTalonConfigs);        
        
        CANcoderConfiguration intakeCANcoderConfigs = new CANcoderConfiguration();
        intakeCANcoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        elbowEncoder.getConfigurator().apply(intakeCANcoderConfigs);
        wristEncoder.getConfigurator().apply(intakeCANcoderConfigs);
    }

    public double elbowGetRadians(){
        //angle ret 0-1
        double raw_angle = Math.PI * 2 * (elbowEncoder.getAbsolutePosition().getValueAsDouble() - Forearm.ELBOW_ENCODER_OFFSET);    // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }

     public double elbowGetDegrees(){
        //angle ret 0-1 //wait it doesnt u gotta scale yup
        double raw_angle = 360 * (elbowEncoder.getAbsolutePosition().getValueAsDouble() - Forearm.ELBOW_ENCODER_OFFSET);  // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }
    
    public double wristGetRadians(){
        //angle ret 0-1
        double raw_angle = Math.PI * 2 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Forearm.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    public double wristGetDegrees(){
        //angle ret 0-1
        double raw_angle = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Forearm.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    public double elbowGetCurrent(){
        return elbow.getTorqueCurrent().getValueAsDouble();
    }
    // public double wristGetCurrent(){
    //     return wrist.get
    // }

    // ret OUTPUT voltage
    public double elbowGetVoltage(){
        return elbow.getMotorVoltage().getValueAsDouble();
    }

    // ret OUTPUT voltage
    // public double wristGetVoltage(){
    //     return wrist.getMotorVoltage().getValueAsDouble();
    // }

    public void elbowSetAngle(double angle, double powerPercent){   // Assuming powerPercent is not signed (because it's a user input)
        if (powerPercent > 100) powerPercent = 100;
        if (powerPercent < 0) powerPercent = 0;

        double power = 12 * powerPercent / 100;
        if(angle > Forearm.ELBOW_HIGH_BOUND) angle = Forearm.ELBOW_HIGH_BOUND;
        if(angle < Forearm.ELBOW_LOW_BOUND) angle = Forearm.ELBOW_LOW_BOUND;
        double error = (angle - elbowGetDegrees())/(Forearm.ELBOW_HIGH_BOUND - Forearm.ELBOW_LOW_BOUND);
        double gravity = k_G * Math.sin(elbowGetRadians() + Forearm.ELBOW_ENCODER_OFFSET_FROM_ZERO);
        
        if(angle > elbowGetDegrees()){
            elbow.setVoltage((k_P * error * power) - gravity);
        }
        else{
            elbow.setVoltage(((k_P * error) * power));
        }
        // SmartDashboard.putNumber("Elbow error",  (k_P * error * power));
    } 

    public void wristSetAngle(double angle, double powerPercent){   // Assuming powerPercent is not signed (because it's a user input)
        if (powerPercent > 100) powerPercent = 100;
        if (powerPercent < 0) powerPercent = 0;

        double power = 12 * powerPercent / 100;
        if(angle > Forearm.WRIST_HIGH_BOUND) angle = Forearm.WRIST_HIGH_BOUND;
        if(angle < Forearm.WRIST_LOW_BOUND) angle = Forearm.WRIST_LOW_BOUND;
        double error = (angle - wristGetDegrees())/(Forearm.WRIST_HIGH_BOUND - Forearm.WRIST_LOW_BOUND);
        double gravity = k_G * Math.sin(wristGetRadians() + Forearm.WRIST_ENCODER_OFFSET_FROM_ZERO);

        if(angle > wristGetDegrees()){
            wrist.setVoltage((k_P * error * power) - gravity);
        }
        else{
            wrist.setVoltage(((k_P * error) * power));
        }
         SmartDashboard.putNumber("Elbow error",  (k_P * error * power));
    }
    public void gravityCompensation(){
        elbow.set(k_G * Math.sin(wristGetRadians() + Forearm.WRIST_ENCODER_OFFSET_FROM_ZERO));
    }

    public ArmState getArmState(){
        return state;
    }

    
    public void setArmState(ArmState state){
        this.state = state;
    }


    @Override
    //We need to change getArmState() to getDesiredPositions as per menotr comments
    //getArmState should be current value/position and getDesiredPosition is what the arm is moving too
    public void periodic(){
        if(getArmState() == ArmState.NEUTRAL){
            //go to netural position here
            roller.roll(Roller.ROLLING_SPEED);
            if (elbowGetDegrees() > GroundNeutralPerimeterConstants.UPPER_MOTION_ELBOW_ANGLE){
                forearm.elbowSetAngle(GroundNeutralPerimeterConstants.UPPER_MOTION_ELBOW_ANGLE, GroundNeutralPerimeterConstants.ELBOW_POWER_PERCENT);
                forearm.wristSetAngle(GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE, GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
            }
            else{
                forearm.elbowSetAngle(GroundNeutralPerimeterConstants.LOWER_MOTION_ELBOW_ANGLE, GroundNeutralPerimeterConstants.ELBOW_POWER_PERCENT);
                forearm.wristSetAngle(GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE, GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
            } 
        }
        else if(getArmState() == ArmState.AMP){
            forearm.elbowSetAngle(AmpScoringPositions.AMP_ELBOW_ANGLE,100);
            forearm.wristSetAngle(AmpScoringPositions.AMP_WRIST_ANGLE,100);
            roller.roll(Roller.ROLLING_SPEED);
        }
        else if(getArmState() == ArmState.SOURCE){
            forearm.elbowSetAngle(SourceScoringPositions.SOURCE_ELBOW_ANGLE,100);   // Change to constants when we have time
            forearm.wristSetAngle(SourceScoringPositions.SOURCE_WRIST_ANGLE,100);   // Change to constants when we have time
            roller.roll(-Roller.ROLLING_SPEED);
        }

    }
}
