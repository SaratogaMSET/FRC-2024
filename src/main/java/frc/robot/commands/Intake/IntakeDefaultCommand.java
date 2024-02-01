package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;

public class IntakeDefaultCommand extends Command{
    ArmSubsystem armSubsystem;
    ArmSubsystemIO armIO;
    DoubleSupplier shoulderAngleModifier;
    DoubleSupplier wristAngleModifier;
    double speed;
    
    public IntakeDefaultCommand(ArmSubsystem armSubsystem, ArmSubsystemIO armIO, DoubleSupplier wristAngleModifier, DoubleSupplier shoulderAngleModifier, double speed){
        this.armSubsystem = armSubsystem;
        this.armIO = armIO;
        this.wristAngleModifier = wristAngleModifier;
        this.shoulderAngleModifier = shoulderAngleModifier;
        this.speed = speed;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute(){
        // Currently here to demonstrate Sim capabilities, will update (put in simulationPeriodic) once merged with Ansh + Anvi's subsystem structure changes
        new ParallelCommandGroup(new ManualWrist(armIO, speed, wristAngleModifier.getAsDouble() * 90), new ManualShoulder(armIO, speed, shoulderAngleModifier.getAsDouble() * 90)).schedule();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

    @Override 
    public void end(boolean interrupted){}
}
