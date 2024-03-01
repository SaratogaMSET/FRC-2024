 package frc.robot.commands.Intake;

 import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

 public class RollerCommand extends Command {
     IntakeSubsystem intake;
     double voltage;
     double speed = 0;
     boolean useIRGate = true;

     public RollerCommand(IntakeSubsystem intake, double voltage){
        this.intake = intake;
        this.voltage = voltage;
     }

     @Override
     public void execute(){
        intake.setRollerVoltage(voltage);
     }

     public void end(boolean interrupted) {
        intake.setRollerVoltage(0);
     }

     @Override 
     public boolean isFinished(){
        return true;
     }
 }
 
