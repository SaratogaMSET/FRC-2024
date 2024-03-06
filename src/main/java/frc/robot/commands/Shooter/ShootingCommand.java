package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootingCommand extends Command{
    ShooterSubsystem shooterSubsystem;
    double turretAngleDegrees;
    double pivotAngleDegrees;
    double shooterVoltage;
    public ShootingCommand(ShooterSubsystem shooterSubsystem,double shooterVoltage, double turretAngleDegrees, double pivotAngleDegrees){
        this.pivotAngleDegrees = pivotAngleDegrees;
        this.turretAngleDegrees = turretAngleDegrees;
        this.shooterVoltage = shooterVoltage;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    shooterSubsystem.setTurretPDF(turretAngleDegrees, 0.0);
    shooterSubsystem.setPivotPDF(pivotAngleDegrees, 0.0);
    shooterSubsystem.setShooterVoltage(shooterVoltage);
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    return false;
  }
}
