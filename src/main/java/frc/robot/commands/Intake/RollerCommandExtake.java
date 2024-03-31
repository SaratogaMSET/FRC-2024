package frc.robot.commands.Intake;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;

public class RollerCommandExtake extends Command {
   RollerSubsystem roller;
   double voltage;
   double speed = 0;
   boolean ampIntake = true;
   DoubleSupplier shoulderPosition;

   public RollerCommandExtake(RollerSubsystem roller, double voltage, boolean ampIntake, DoubleSupplier shoulderPosition){
      this.roller = roller;
      this.voltage = voltage;
      addRequirements(roller);
   }

   @Override
   public void initialize(){
        roller.setIntakeFeederMode(false);
        roller.setShooterFeederMode(true);
        roller.setShooterFeederVoltage(voltage);
   }
   @Override
   public void execute(){
      roller.setIntakeFeederVoltage(voltage);
      roller.setShooterFeederVoltage(2);
      // previousIntakeTriggered = intakeTriggered;
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
