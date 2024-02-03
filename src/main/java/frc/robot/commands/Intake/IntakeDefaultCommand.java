package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystem;

public class IntakeDefaultCommand extends Command{
    ArmSubsystem armSubsystem;
    ArmState armState;
    
    public IntakeDefaultCommand(ArmSubsystem armSubsystem, ArmState armState){
        this.armSubsystem = armSubsystem;
        this.armState = armState;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        armSubsystem.setArmState(armState);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override 
    public void end(boolean interrupted){}
}
