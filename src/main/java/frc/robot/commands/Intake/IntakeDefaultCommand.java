package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;

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
        // Currently here to demonstrate Sim capabilities, will update (put in simulationPeriodic) once merged with Ansh + Anvi's subsystem structure changes
        armSubsystem.setArmState(armState);
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override 
    public void end(boolean interrupted){}
}
