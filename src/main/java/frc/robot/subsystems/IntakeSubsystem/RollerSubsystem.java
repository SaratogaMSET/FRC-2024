package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;  
import frc.robot.Constants.IntakeSubsystem.Roller;
// import com.ctre.phoenix6.motorcontrol.SupplyCurrentLimitConfiguration;   // TODO: Fix current limiting


public class RollerSubsystem extends SubsystemBase{
    TalonFX roller; 
    DigitalInput irGate = new DigitalInput(Roller.IR_GATE);    
    


    public RollerSubsystem(){
        roller = new TalonFX(Roller.ROLLER_MOTOR, "Placeholder"); 
        irGate = new DigitalInput(Roller.IR_GATE); 
        roller.setNeutralMode(Roller.ROLLER_NEUTRAL_MODE);
        // Set motor output configs for configuring deadband
        MotorOutputConfigs rollerTalonOutputConfigs = new MotorOutputConfigs();
        TalonFXConfiguration rollerTalonConfigs = new TalonFXConfiguration();
        rollerTalonOutputConfigs.DutyCycleNeutralDeadband = Roller.NEUTRAL_VOLTAGE;    // TODO: Tune  https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode        
        rollerTalonConfigs.Slot0.kP = 0.0; 
        rollerTalonConfigs.Slot0.kI = 0.0;
        rollerTalonConfigs.Slot0.kD = 0.0;
        rollerTalonConfigs.Slot0.kV = 0.0;
        rollerTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0;//change later
        rollerTalonConfigs.withMotorOutput(rollerTalonOutputConfigs);
        
        roller.getConfigurator().apply(rollerTalonConfigs);
    }

    public void roll(double speed){
        roller.set(speed);
    }

    public double getSpeed(){
        return roller.get();
    } 

    public boolean acquired(){
        return irGate.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Is Object in Roller", acquired()); //ObjectInRoller for 1 ring
    }
}
