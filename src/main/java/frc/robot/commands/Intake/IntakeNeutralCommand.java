package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeNeutralCommand extends Command{
    IntakeSubsystem intakeSubsystem;
    double shoulderAngle;
    double wristAngle;

    boolean intakeHasZeroed;
    boolean previousIntakeHasZeroed;
    
    public IntakeNeutralCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize(){
        intakeHasZeroed = false;
    }
    
    @Override
    public void execute(){
        intakeSubsystem.setAngleShoulder(Neutral.SHOULDER_ANGLE);
        // intakeSubsystem.setWristVoltage(0.0);
        // if (!intakeSubsystem.getHallEffect()) {
        //     intakeSubsystem.setWristVoltage(-1);
        // }
        if(!intakeHasZeroed && ((intakeSubsystem.wrist.motor.getOutputCurrent() < 20) || !intakeSubsystem.getHallEffect())){ //TODO: Test current limit
            intakeSubsystem.setWristVoltage(-1);
        }
        else{
            intakeHasZeroed = true;
            if(intakeHasZeroed && !previousIntakeHasZeroed) intakeSubsystem.wrist.manualHallEffectReset();
            intakeSubsystem.setWristVoltage(0.0);
            
        }
        previousIntakeHasZeroed = intakeHasZeroed;
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
