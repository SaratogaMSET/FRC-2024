package frc.robot.commands.Intake;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;

public class RollerDefaultCommand extends Command {
   RollerSubsystem roller;
   double speed = 0;
   boolean ampIntake = true;
   boolean previousIntakeTriggered = false;
   DoubleSupplier shoulderPosition;

   public RollerDefaultCommand(RollerSubsystem roller, DoubleSupplier shoulderPosition){
      this.roller = roller;
      this.shoulderPosition = shoulderPosition;
      addRequirements(roller);
   }

   @Override
   public void initialize(){
      
   }
   @Override
   public void execute(){
      if(Math.abs(shoulderPosition.getAsDouble() - Neutral.SHOULDER_ANGLE) < 0.1 && roller.getIntakeBeamBreak()){
         roller.setIntakeFeederVoltage(-0.7);
      }else{
         roller.setIntakeFeederVoltage(0);
      }
      roller.setShooterFeederVoltage(0);
   }

   public void end(boolean interrupted) {
      roller.setIntakeFeederVoltage(0);
      roller.setShooterFeederVoltage(0);
   }

   @Override 
   public boolean isFinished(){
      return false;
   }
}
