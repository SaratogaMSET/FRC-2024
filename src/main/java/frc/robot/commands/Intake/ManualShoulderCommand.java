package frc.robot.commands.Intake;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO.ArmSubsystemIOInputs;

public class ManualShoulderCommand extends Command{
    ArmSubsystemIO shoulder;
    ArmSubsystemIOInputs armIOInputs;
    double velocity = 0;
    double angle = 0.0;

    public ManualShoulderCommand(ArmSubsystemIO shoulder, double velocity, double angle){
        this.shoulder = shoulder;
        this.velocity = velocity;
        this.angle = angle;
    }


    @Override
    public void execute(){
        shoulder.shoulderSetAngle(angle, 100);
        shoulder.updateInputs(armIOInputs);
    }
    
    @Override
    public boolean isFinished(){
        return true;
    }
}