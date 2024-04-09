package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeNeutralCommand extends Command{
    IntakeSubsystem intakeSubsystem;
    double shoulderAngle;
    double wristAngle;

    double previousResetTime = Timer.getFPGATimestamp();
    BooleanSupplier gunnerResetWrist;
    public IntakeNeutralCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
        // this.gunnerResetWrist = gunnerResetWrist;
        addRequirements(this.intakeSubsystem);
    }

    @Override
    public void initialize(){
    }
    
    @Override
    public void execute(){
        if(!DriverStation.isTest()){
            intakeSubsystem.setAngleShoulder(Neutral.SHOULDER_ANGLE);
            if(Timer.getFPGATimestamp() - previousResetTime > 3){
                intakeSubsystem.wristIOInputs.previouslyZeroed = false;
            }
            // if(gunnerResetWrist.getAsBoolean()){
            //     intakeSubsystem.wristIOInputs.previouslyZeroed = false;
            // }
            if (intakeSubsystem.getCurrentLimit() && !intakeSubsystem.wristIOInputs.previouslyZeroed && intakeSubsystem.wrist.motor.getEncoder().getPosition() < 0.07) {
                previousResetTime = Timer.getFPGATimestamp();
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
        else{
            intakeSubsystem.setShoulderVoltage(0.0);
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
