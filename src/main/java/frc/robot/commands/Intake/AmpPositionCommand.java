package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.ForearmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;
import frc.robot.Constants.IntakeSubsystem.Forearm.ArmState;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.Constants.IntakeSubsystem.Forearm.AmpScoringPositions;

public class AmpPositionCommand extends Command {
    RollerSubsystem roller;
    ForearmSubsystem forearm;
    double speed = 0;
    boolean useIRGate = true;

    public AmpPositionCommand(RollerSubsystem roller, ForearmSubsystem forearm, double speed){
        this.roller = roller;
        this.forearm = forearm;
        this.speed = speed;
        addRequirements(forearm, roller);
    }

    // Make the manual command a separate command for ease of use in RobotContainer
    // public GroundDeployCommand(RollerSubsystem rollerSubsystem, double speed, boolean IRGate){
    //     this.roller = rollerSubsystem;
    //     this.speed = speed;
    //     this.useIRGate = IRGate;
    //     addRequirements(rollerSubsystem);
    // }
    

    @Override
    public void execute(){
        /*forearm.elbowSetAngle(AmpScoringPositions.AMP_ELBOW_ANGLE,100);
        forearm.wristSetAngle(AmpScoringPositions.AMP_WRIST_ANGLE,100);
        roller.roll(Roller.ROLLING_SPEED);*/
        forearm.setArmState(ArmState.AMP);
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