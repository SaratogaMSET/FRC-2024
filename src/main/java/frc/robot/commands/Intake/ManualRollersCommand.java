 package frc.robot.commands.Intake;

 import edu.wpi.first.wpilibj2.command.Command;
 import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIO;
 import frc.robot.Constants.IntakeSubsystem.Roller;
 import frc.robot.Constants.IntakeSubsystem.Roller.RollerState;

 public class ManualRollersCommand extends Command {
     RollerSubsystemIO roller;
     double speed = 0;
     boolean useIRGate = true;
     RollerState rollerState;

     public ManualRollersCommand(RollerSubsystemIO intake, double speed){
         this.roller = intake;
        // addRequirements(intake); Need to fix
         this.speed = speed;

     }

     @Override
     public void execute(){
        switch (rollerState){
            case INTAKE:
                roller.roll(speed);
                break;
            case OUTAKE:
                roller.roll(-speed);
                break; 
        }
        roller.updateInputs();
     }

     public void end(boolean interrupted) {
         if (roller.acquired()) {
             roller.roll(Roller.HOLD_SPEED);
         } else {
             roller.roll(Roller.NEUTRAL_SPEED);
         }
     }

     @Override 
     public boolean isFinished(){
        return false;
     }
 }
 
