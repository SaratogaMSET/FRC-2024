package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Superstructure.Intake.Roller.RollerSubsystem;
import java.util.function.BooleanSupplier;

public class ShooterNeutral extends Command {
  ShooterSubsystem shooterSubsystem;
  RollerSubsystem rollerSubsystem;
  BooleanSupplier gunnerRevvingSupplier;
  BooleanSupplier intaking;

  public ShooterNeutral(
      ShooterSubsystem shooterSubsystem,
      RollerSubsystem rollerSubsystem,
      BooleanSupplier gunnerRevving,
      BooleanSupplier intaking) {
    this.shooterSubsystem = shooterSubsystem;
    this.rollerSubsystem = rollerSubsystem;
    this.gunnerRevvingSupplier = gunnerRevving;
    this.intaking = intaking;
    addRequirements(shooterSubsystem);
  }
  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}

  public void execute() {
    shooterSubsystem.setTurretProfiled(0, 0);
    if ((rollerSubsystem.getCarriageBeamBreak() || rollerSubsystem.getShooterFeederBeamBreak())
        && !rollerSubsystem.getShooterBeamBreak()) { // Intaking
      shooterSubsystem.setPivotProfiled(Math.toRadians(44), 0);
    } else {
      shooterSubsystem.setPivotProfiled(ShooterPivotConstants.kHigherBound, 0); // WHY BRUH
    }

    // Rev Logic.
    if (gunnerRevvingSupplier.getAsBoolean()) {
      shooterSubsystem.spinShooterMPS(9.0, 0.0);
      if (intaking.getAsBoolean()) {
        shooterSubsystem.setPivotProfiled(44, 0);
      }
    } else if (DriverStation.isAutonomous()) {
      shooterSubsystem.spinShooterMPS(0.0, 0.0); // configure to not brownout
    } else if (DriverStation.isTeleop()) {
      shooterSubsystem.setShooterVoltage(0.0);
    } else {
      shooterSubsystem.setShooterVoltage(0.0); // how did we get here
    }
  }

  public void end(boolean interrupted) {}

  public boolean isFinished() {
    return false;
  }
}
