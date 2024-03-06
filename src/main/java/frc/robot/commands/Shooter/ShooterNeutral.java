package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterNeutral extends Command{
    ShooterSubsystem shooterSubsystem;
    public ShooterNeutral(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    shooterSubsystem.setTurretPDF(0, 0);
    shooterSubsystem.setPivotPDF(ShooterPivotConstants.kHigherBound, 0);
    shooterSubsystem.setShooterVoltage(0.0);
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    return false;
  }
}
