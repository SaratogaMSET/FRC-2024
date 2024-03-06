 package frc.robot.commands.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;

 public class RollerCommand extends Command {
      RollerSubsystem roller;
     double voltage;
     double speed = 0;
     boolean useIRGate = true;
     boolean previousIntakeTriggered = false;

     public RollerCommand(RollerSubsystem roller, double voltage, boolean stopAtWristIR){
         this.roller = roller;
        this.voltage = voltage;
        useIRGate = stopAtWristIR;
        addRequirements(roller);
     }

     @Override
     public void initialize(){
         roller.setShooterFeederMode(true);
         if(roller.getIntakeBeamBreak()){
            roller.setShooterFeederVoltage(voltage);
         }
     }
     @Override
     public void execute(){
        roller.setIntakeFeederVoltage(voltage);
        boolean intakeTriggered = roller.getIntakeBeamBreak();
        if(!useIRGate){
            if(intakeTriggered == false && previousIntakeTriggered == true){
               roller.setShooterFeederVoltage(1.5);
            }
        }
        else{
            roller.setShooterFeederVoltage(0.0);
        }
        previousIntakeTriggered = intakeTriggered;
     }

     public void end(boolean interrupted) {
         roller.setIntakeFeederVoltage(0);
         roller.setShooterFeederVoltage(0);
     }

     @Override 
     public boolean isFinished(){
         if(useIRGate){
            return roller.getIntakeBeamBreak();
         }
         else{
            if(voltage > 0){
               return roller.getShooterBeamBreak();
            }
            else if (voltage == 0){
               return true;
            }
               return false;
         }
     }
 }
 
