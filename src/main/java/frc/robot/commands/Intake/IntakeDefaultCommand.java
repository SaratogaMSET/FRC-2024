package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake.AcutatorConstants.ActuatorState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeDefaultCommand extends Command{
    IntakeSubsystem armSubsystem;
    ActuatorState armState;
    
    public IntakeDefaultCommand(IntakeSubsystem armSubsystem, ActuatorState armState){
        this.armSubsystem = armSubsystem;
        this.armState = armState;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize(){
        armSubsystem.setArmState(armState);
    }
    
    @Override
    public void execute(){
        armSubsystem.runArm();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override 
    public void end(boolean interrupted){}
}
