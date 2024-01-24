package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.ForearmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;
import frc.robot.Constants.IntakeSubsystem.Forearm.ArmState;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.Constants.IntakeSubsystem.Forearm.SourceScoringPositions;

public class SourceIntakeCommand extends Command {
    RollerSubsystem roller;
    ForearmSubsystem forearm;
    double speed = 0;
    boolean useIRGate = true;

    public SourceIntakeCommand(RollerSubsystem roller, ForearmSubsystem forearm, double speed){
        this.roller = roller;
        this.forearm = forearm;
        this.speed = speed;
        addRequirements(forearm, roller);
    }
    
    @Override
    public void execute(){
        forearm.elbowSetAngle(SourceScoringPositions.SOURCE_ELBOW_ANGLE,100);   // Change to constants when we have time
        forearm.wristSetAngle(SourceScoringPositions.SOURCE_WRIST_ANGLE,100);   // Change to constants when we have time
        roller.roll(-Roller.ROLLING_SPEED);
        forearm.setArmState(ArmState.SOURCE);
    }

    @Override
    public void end(boolean interrupted){
        if (roller.acquired()) {
            roller.roll(Roller.HOLD_SPEED);
        } else {
            roller.roll(Roller.NEUTRAL_SPEED);
        }
    }

    @Override
    public boolean isFinished(){
        return roller.acquired();
    }
}