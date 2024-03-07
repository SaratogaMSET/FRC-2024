package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.Intake.Roller;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterParameters;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShootingCommand extends Command{
    ShooterSubsystem shooterSubsystem;
    RollerSubsystem roller;
    double turretAngleDegrees;
    double pivotAngleDegrees;
    double shooterRPM;
    public ShootingCommand(ShooterSubsystem shooterSubsystem, RollerSubsystem roller, double shooterRPM, double turretAngleDegrees, double pivotAngleDegrees){
        this.pivotAngleDegrees = pivotAngleDegrees;
        this.turretAngleDegrees = turretAngleDegrees;
        this.shooterRPM = shooterRPM;
        this.shooterSubsystem = shooterSubsystem;
        this.roller = roller;
        addRequirements(shooterSubsystem);
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    shooterSubsystem.setTurretPDF(turretAngleDegrees, 0.0);
    shooterSubsystem.setPivotPDF(pivotAngleDegrees, 0.0);
    shooterSubsystem.spinShooter(shooterRPM);
    if(Math.abs(shooterRPM - shooterSubsystem.rpmShooterAvg()) < ShooterFlywheelConstants.tolerance){
      roller.setShooterFeederVoltage(12);
    }
  }
  public void end(boolean interrupted) {
    shooterSubsystem.setTurretPDF(0.0, 0.0);
    shooterSubsystem.setPivotPDF(0.0, 0.0);
    shooterSubsystem.setShooterVoltage(0.0);
    roller.setShooterFeederVoltage(0);
  }
  public boolean isFinished() {
    return false;
  }
}
