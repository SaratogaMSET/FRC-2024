package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Shooter.Angling.AnglingSubsystem;

public class ShooterNeutral extends Command{
    ShooterSubsystem shooterSubsystem;
    AnglingSubsystem anglingSubsystem;

    public ShooterNeutral(ShooterSubsystem shooterSubsystem, AnglingSubsystem anglingSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.anglingSubsystem = anglingSubsystem;
        addRequirements(shooterSubsystem);
        addRequirements(anglingSubsystem);
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    anglingSubsystem.setTurretPDF(0, 0);
    anglingSubsystem.setPivotPDF(ShooterPivotConstants.kHigherBound, 0);
    shooterSubsystem.setShooterVoltage(0.0);
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    return false;
  }
}
