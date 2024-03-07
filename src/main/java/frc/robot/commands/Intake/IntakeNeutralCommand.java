package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeNeutralCommand extends Command{
    IntakeSubsystem intakeSubsystem;
    double shoulderAngle;
    double wristAngle;
    
    public IntakeNeutralCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        intakeSubsystem.setAngleShoulder(Neutral.SHOULDER_ANGLE);
        if (!intakeSubsystem.getHallEffect()) {
            intakeSubsystem.setWristVoltage(-1);
        }
        else{
            intakeSubsystem.setWristVoltage(0.0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override 
    public void end(boolean interrupted){
        // intakeSubsystem.setVoltages(0.0, 0.0);
    }
}
