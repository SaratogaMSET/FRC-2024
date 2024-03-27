package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.Intake.Roller;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterNeutral extends Command{
    ShooterSubsystem shooterSubsystem;
    RollerSubsystem rollerSubsystem;

    public ShooterNeutral(ShooterSubsystem shooterSubsystem, RollerSubsystem rollerSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.rollerSubsystem = rollerSubsystem;
        addRequirements(shooterSubsystem);
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    shooterSubsystem.setTurretProfiled(0, 0);

    if(rollerSubsystem.getCarriageBeamBreak()){
      shooterSubsystem.setPivotProfiled(Math.toDegrees(44), 0);
    }else{
      shooterSubsystem.setPivotProfiled(ShooterPivotConstants.kHigherBound, 0);
    }

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
