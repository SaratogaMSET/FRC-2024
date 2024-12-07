package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Superstructure.Intake.Roller.RollerSubsystem;

public class AutoPreload extends Command {
  ShooterSubsystem shooterSubsystem;
  RollerSubsystem rollerSubsystem;
  double minimumRPM;
  boolean feedingStarted = false;

  public AutoPreload(
      ShooterSubsystem shooterSubsystem, RollerSubsystem rollerSubsystem, double minimumRPM) {
    this.shooterSubsystem = shooterSubsystem;
    this.rollerSubsystem = rollerSubsystem;
    this.minimumRPM = minimumRPM;
    addRequirements(shooterSubsystem);
  }

  public void initialize() {}

  public void execute() {
    shooterSubsystem.setTurretProfiled(0, 0);
    shooterSubsystem.setPivotProfiled(ShooterPivotConstants.kHigherBound, 0);
    shooterSubsystem.setShooterVoltage(12);
    if (shooterSubsystem.rpmShooterAvg() > minimumRPM) {
      feedingStarted = true;
      rollerSubsystem.setShooterFeederVoltage(12);
    }
  }

  public void end(boolean interrupted) {
    shooterSubsystem.setTurretVoltage(0);
    shooterSubsystem.setPivotVoltage(0);
    shooterSubsystem.setShooterVoltage(0);
    rollerSubsystem.setShooterFeederVoltage(0);
  }

  public boolean isFinished() {
    return feedingStarted && !rollerSubsystem.getShooterBeamBreak();
  }
}
