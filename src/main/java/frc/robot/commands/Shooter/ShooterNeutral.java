package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterAnglerConstants;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterNeutral extends Command{
    ShooterSubsystem shooterSubsystem;
    BooleanSupplier feedFromIntake;
    public ShooterNeutral(ShooterSubsystem shooterSubsystem , BooleanSupplier feedFromIntake){
        this.shooterSubsystem = shooterSubsystem;
        this.feedFromIntake = feedFromIntake;
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    shooterSubsystem.setPhiPDF(0, 0);
    shooterSubsystem.setThetaPDF(ShooterAnglerConstants.kNeutralDegrees, 0);
    if(feedFromIntake.getAsBoolean() && Math.abs(shooterSubsystem.phiDegrees()) < 3 && Math.abs(shooterSubsystem.thetaDegrees() - ShooterAnglerConstants.kNeutralDegrees) < 3){
        shooterSubsystem.setFeederVoltage(ShooterFeederConstants.feedVoltage); //TODO: Define Feeding Voltage & Feeding Angle
    }
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    return false;
  }
}
