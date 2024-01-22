package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.robot.Constants.IntakeSubsystem.Actuator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ActuatorSubsystem {
    TalonFX elbow = new TalonFX(Actuator.INTAKE_ELBOW_MOTOR, "Placeholder"); 
    TalonFX wrist = new TalonFX(Actuator.INTAKE_WRIST_MOTOR, "Placeholder");
    CANcoder elbowEncoder = new CANcoder(Actuator.INTAKE_ELBOW_ENCODER,  "Placeholder");
    CANcoder wristEncoder = new CANcoder(Actuator.INTAKE_WRIST_ENCODER,  "Placeholder");

    double previousError = 0;   // Move to constants, preferably in nested class within Actuator class
    public double k_G = 0; 
    double k_P = 0; 
    double k_D = 0.000;
    double k_I = 0.000;
    double errorDT;
    double prevError;

    PIDController controller = new PIDController(k_P, 0.0 ,k_D);
    //fix current limiting nvm im the goat
        /*
    SupplyCurrentLimitConfiguration ActuatorLimit = new SupplyCurrentLimitConfiguration(
            true, 
            Constants.Drivetrain.driveContinuousCurrentLimit, 
            GroundIntake.currentLimit, 
            Constants.Drivetrain.drivePeakCurrentDuration);*/
    public ActuatorSubsystem(){
        elbow.setNeutralMode(Actuator.ACTUATOR_NEUTRAL_MODE);
        wrist.setNeutralMode(Actuator.ACTUATOR_NEUTRAL_MODE);

        // Set motor output configs for configuring deadband
        MotorOutputConfigs intakeTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration intakeTalonConfigs = new TalonFXConfiguration();
        intakeTalonOutputConfigs.DutyCycleNeutralDeadband = Actuator.NEUTRAL_VOLTAGE;    // TODO: Tune  https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode        
        //elbow.configNeutralDeadband(0.0);//No fing idea // Here: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
        intakeTalonConfigs.Slot0.kP = 1;
        intakeTalonConfigs.Slot0.kI = 0;
        intakeTalonConfigs.Slot0.kD = 10;
        intakeTalonConfigs.Slot0.kV = 2;
        intakeTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0;//change later
        intakeTalonConfigs.withMotorOutput(intakeTalonOutputConfigs);
        
        elbow.getConfigurator().apply(intakeTalonConfigs);
        wrist.getConfigurator().apply(intakeTalonConfigs);
        
        CANcoderConfiguration intakeCANcoderConfigs = new CANcoderConfiguration();
        intakeCANcoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;

        elbowEncoder.getConfigurator().apply(intakeCANcoderConfigs);
        wristEncoder.getConfigurator().apply(intakeCANcoderConfigs);
    }

    public double elbowGetRadians(){
        //angle ret 0-1
        double raw_angle = Math.PI * 2 * (elbowEncoder.getAbsolutePosition().getValueAsDouble() - Actuator.ELBOW_ENCODER_OFFSET);    // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }

     public double elbowGetDegrees(){
        //angle ret 0-1 //wait it doesnt u gotta scale yup
        double raw_angle = 360 * (elbowEncoder.getAbsolutePosition().getValueAsDouble() - Actuator.ELBOW_ENCODER_OFFSET);  // Assuming encoder offset is in native units (rotations [0, 1))
        return raw_angle;
    }
    
    public double wristGetRadians(){
        //angle ret 0-1
        double raw_angle = Math.PI * 2 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Actuator.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    public double wristGetDegrees(){
        //angle ret 0-1
        double raw_angle = 360 * (wristEncoder.getAbsolutePosition().getValueAsDouble() - Actuator.WRIST_ENCODER_OFFSET);
        return raw_angle;
    }

    public double elbowGetCurrent(){
        return elbow.getTorqueCurrent().getValueAsDouble();
    }
    public double wristGetCurrent(){
        return wrist.getTorqueCurrent().getValueAsDouble();
    }

    // ret OUTPUT voltage
    public double elbowGetVoltage(){
        return elbow.getMotorVoltage().getValueAsDouble();
    }

    // ret OUTPUT voltage
    public double wristGetVoltage(){
        return wrist.getMotorVoltage().getValueAsDouble();
    }

    public void elbowSetAngle(double angle, double powerPercent){   // Assuming powerPercent is not signed (because it's a user input)
        if (powerPercent > 100) powerPercent = 100;
        if (powerPercent < 0) powerPercent = 0;

        double power = 12 * powerPercent / 100;
        if(angle > Actuator.ELBOW_HIGH_BOUND) angle = Actuator.ELBOW_HIGH_BOUND;
        if(angle < Actuator.ELBOW_LOW_BOUND) angle = Actuator.ELBOW_LOW_BOUND;
        double error = (angle - elbowGetDegrees())/(Actuator.ELBOW_HIGH_BOUND - Actuator.ELBOW_LOW_BOUND);
        double gravity = k_G * Math.sin(elbowGetRadians() + Actuator.ELBOW_ENCODER_OFFSET_FROM_ZERO);
        
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
        if(angle > Actuator.WRIST_HIGH_BOUND) angle = Actuator.WRIST_HIGH_BOUND;
        if(angle < Actuator.WRIST_LOW_BOUND) angle = Actuator.WRIST_LOW_BOUND;
        double error = (angle - wristGetDegrees())/(Actuator.WRIST_HIGH_BOUND - Actuator.WRIST_LOW_BOUND);
        double gravity = k_G * Math.sin(wristGetRadians() + Actuator.WRIST_ENCODER_OFFSET_FROM_ZERO);

        if(angle > wristGetDegrees()){
            wrist.setVoltage((k_P * error * power) - gravity);
        }
        else{
            wrist.setVoltage(((k_P * error) * power));
        }
        // SmartDashboard.putNumber("Elbow error",  (k_P * error * power));
    }
}
