/**
 * Anvi's Subsystem
 */

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.subsystems.IntakeSubsystem.ForearmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

public class ManualIntakeCommand extends Command {
    
    RollerSubsystem roller;
    double speed = 0;
    public ManualIntakeCommand(RollerSubsystem rollerSubsystem, double speed){
        this.roller = rollerSubsystem;
        this.speed = speed;
        addRequirements(rollerSubsystem);
    }
    
    @Override
    public void execute(){
        roller.roll(speed);
    }

    @Override
    public void end(boolean interrupted){
        if (roller.acquired()) {
            roller.roll(Roller.HOLD_SPEED);
        } else {
            roller.roll(Roller.NEUTRAL_SPEED);
        }
    }

    @Override
    public boolean isFinished(){
        return roller.acquired();
    }
}