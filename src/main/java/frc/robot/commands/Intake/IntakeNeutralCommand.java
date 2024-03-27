package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
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
        if (intakeSubsystem.getCurrentLimit() && !intakeSubsystem.wristIOInputs.previouslyZeroed && intakeSubsystem.wrist.motor.getEncoder().getPosition() < 0.07) {
            intakeSubsystem.wristIOInputs.previouslyZeroed = true;
            intakeSubsystem.setWristVoltage(0.0);
            intakeSubsystem.wrist.motor.getEncoder().setPosition(0.0);
        }
        else if(intakeSubsystem.wristIOInputs.previouslyZeroed) {
            intakeSubsystem.setAngleWrist(0);
        }
        else {
            intakeSubsystem.setWristVoltage(-2); //-1.5
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
