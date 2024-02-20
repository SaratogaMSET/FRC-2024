package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretSubsystem;

public class ShooterNeutral extends Command{
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;
    BooleanSupplier feedFromIntake;
    public ShooterNeutral(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem, BooleanSupplier feedFromIntake){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.feedFromIntake = feedFromIntake;
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    turretSubsystem.setAnglePDF(0, 0);
    if(feedFromIntake.getAsBoolean() && Math.abs(turretSubsystem.angleDegrees()) < 3){
        shooterSubsystem.setFeederVoltage(0); //TODO: Define Feeding Voltage & Feeding Angle
        shooterSubsystem.setAnglePDF(0, 0);
    }
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    return false;
  }
}
