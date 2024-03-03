 package frc.robot.commands.Intake;

 import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

 public class RollerCommand extends Command {
     IntakeSubsystem intake;
     ShooterSubsystem shooter;
     double voltage;
     double speed = 0;
     boolean useIRGate = true;

     public RollerCommand(IntakeSubsystem intake, ShooterSubsystem shooter, double voltage, boolean stopAtWristIR){
        this.intake = intake;
        this.voltage = voltage;
        this.shooter = shooter;
        useIRGate = stopAtWristIR;
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
         if(useIRGate){
            return intake.getBeamBreak();
         }
         else{
            return shooter.beamBreak();
         }
     }
 }
 
