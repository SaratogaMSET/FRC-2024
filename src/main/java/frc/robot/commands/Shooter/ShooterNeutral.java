package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
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
    shooterSubsystem.setTurretProfiled(0, 0);
    shooterSubsystem.setPivotProfiled(ShooterPivotConstants.kHigherBound, 0);
    if(DriverStation.isTeleop()){
      shooterSubsystem.setShooterVoltage(0.0);
    }
    else if(DriverStation.isAutonomous()){
      shooterSubsystem.spinShooterMPS(9.0);
    }
    else{
      shooterSubsystem.setShooterVoltage(0.0); //how did we get here
    }
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    return false;
  }
}
