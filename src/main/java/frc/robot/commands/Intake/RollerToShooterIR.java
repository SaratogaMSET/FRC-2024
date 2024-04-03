package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class RollerToShooterIR extends Command{

    RollerSubsystem roller;
    ShooterSubsystem shooter;
    double voltage;
    
    public RollerToShooterIR(RollerSubsystem roller, ShooterSubsystem shooter, double voltage){
        this.roller = roller;
        this.shooter = shooter;
        this.voltage = voltage;
    }

    @Override
    public void execute(){
        roller.setIntakeFeederVoltage(voltage);
        shooter.anglingDegrees(0, 44);
        roller.setShooterFeederVoltage(1.2);
    }

    @Override
    public void end(boolean interrupted) {
        roller.setIntakeFeederVoltage(0);
        roller.setShooterFeederVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return roller.getShooterBeamBreak(); // True, we stop rolling!
    }

}
