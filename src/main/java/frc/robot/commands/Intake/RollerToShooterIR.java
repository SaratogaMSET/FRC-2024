package frc.robot.commands.Intake;

import org.littletonrobotics.junction.Logger;

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

        addRequirements(roller, shooter);
    }
    @Override
    public void initialize() {
        // roller.setIntakeFeederMode(true);
    }
    @Override
    public void execute(){
        if(roller.getCarriageBeamBreak()){
            roller.setIntakeFeederVoltage(2);
        }
        else{
            roller.setIntakeFeederVoltage(voltage);
        }
        shooter.setPivotProfiled(Math.toRadians(44), 0);
        roller.setShooterFeederVoltage(2);
        
    }

    @Override
    public void end(boolean interrupted) {
        roller.setIntakeFeederVoltage(0);
        roller.setShooterFeederVoltage(0);
    }


    @Override
    public boolean isFinished() {
        Logger.recordOutput("rollerBeamBreak", roller.getShooterBeamBreak());
        return roller.getShooterBeamBreak(); // True, we stop rolling!
    }

}
