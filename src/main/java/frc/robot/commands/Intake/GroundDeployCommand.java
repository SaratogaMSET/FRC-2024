package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.ActuatorSubsystem;

public class ManualIntakeCommand extends Command {
    
    RollerSubsystem roller;
    double speed = 0;
    boolean useIRGate = true;
    public ManualIntakeCommand(RollerSubsystem rollerSubsystem, double speed){
        this.roller = rollerSubsystem;
        this.speed = speed;
        addRequirements(rollerSubsystem);
    }
    public ManualIntakeCommand(RollerSubsystem rollerSubsystem, double speed, boolean IRGate){
        this.roller = rollerSubsystem;
        this.speed = speed;
        this.useIRGate = IRGate;
        addRequirements(rollerSubsystem);
    }
    
    @Override
    public void execute(){
        roller.set_intake(speed);
    }

    @Override
    public void end(boolean interrupted){
        if(roller.objectInRoller()){
            roller.set_intake(0.03);
        }
        else{
            roller.set_intake(0);
        }
    }

    @Override
    public boolean isFinished(){
        if(speed>0 && useIRGate){
            return roller.objectInRoller();
        }
        return false;
    }
}