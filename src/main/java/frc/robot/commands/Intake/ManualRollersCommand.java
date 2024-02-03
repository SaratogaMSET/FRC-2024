 package frc.robot.commands.Intake;

 import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
 import frc.robot.Constants.IntakeSubsystem.Roller.RollerState;

 public class ManualRollersCommand extends Command {
     RollerSubsystem roller;
     double speed = 0;
     boolean useIRGate = true;
     RollerState rollerState;

     public ManualRollersCommand(RollerSubsystem roller, RollerState rollerState){
         this.roller = roller;
         this.rollerState = rollerState;

        addRequirements(roller);
     }

     @Override
     public void execute(){
        roller.setRollerState(rollerState);
     }

     public void end(boolean interrupted) {
         if (roller.neutralHold()) {
             roller.setRollerState(RollerState.HOLD);
         } else {
             roller.setRollerState(RollerState.NEUTRAL);
         }
     }

     @Override 
     public boolean isFinished(){
        return true;
     }
 }
 
