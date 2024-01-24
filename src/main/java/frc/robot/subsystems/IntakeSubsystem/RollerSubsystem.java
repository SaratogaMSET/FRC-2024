package frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystem.Actuator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;  
import com.ctre.phoenix6.motorcontrol.SupplyCurrentLimitConfiguration;


public class RollerSubsystem extends SubsystemBase{
    TalonFX roller = new TalonFX(Roller.ROLLER_MOTOR, "Placeholder"); 
    DigitalInput irGate = new DigitalInput(Actuator.IR_GATE);    
    SupplyCurrentLimitConfiguration IntakeLimit = new SupplyCurrentLimitConfiguration(
            true,
            Roller.intakeContinousCurrentLimit,
            Roller.intakePeakCurrentLimit,
            0.1);


    public RollerSubsystem(){
        roller.configSupplyCurrentLimit(IntakeLimit);
        roller.setNeutralMode(Roller.Brake);
    }

    public void roll(double speed){
        roller.set(speed);
    }


   @Override
    public void periodic(){
        SmartDashboard.putBoolean("Is Object in Roller", acquired()); //ObjectInRoller for 1 ring
    }
    public boolean acquired(){


        return irGate.get();
    }
    
    
    
    
}
