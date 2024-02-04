package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIO.RollerSubsystemIOInputs;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.Constants.IntakeSubsystem.Roller.RollerState;

public class RollerSubsystem extends SubsystemBase {
    RollerSubsystemIO roller;
    double rollerVelocity = 0.0;
    RollerState rollerState;
    RollerSubsystemIOInputs rollerIOInputs = new RollerSubsystemIOInputs();

    public RollerSubsystem(RollerSubsystemIO roller) {
        this.roller = roller;
    }

    /**
     * Using the current rollerState, run the roller
     */
    public void runRoller (){
         if (rollerState == null) {
            System.out.println("WARNING: RollerState not set!");
        } else {
            switch (rollerState) {
                case SPEAKER_INTAKE:
                    if(!roller.shooterIR()) // allow rolling as long as ring is not at shooter IR gate
                        rollerVelocity = Roller.ROLLING_SPEED;
                    else 
                        rollerVelocity = Roller.HOLD_SPEED;
                    break;
                case AMP_INTAKE:
                    if(!roller.wristIR())
                        rollerVelocity = Roller.ROLLING_SPEED;
                    else
                        rollerVelocity = rollerVelocity * 0.5 <= Roller.HOLD_SPEED ? Roller.HOLD_SPEED : rollerVelocity * 0.5;  // If hold speed is overkill when testing, change to relying on brake mode --- set velocity = 0
                case OUTTAKE:
                    roller.roll(-Roller.ROLLING_SPEED);
                    break;
                case NEUTRAL:
                    roller.roll(Roller.NEUTRAL_SPEED);
                    break;
                case HOLD:
                    roller.roll(Roller.HOLD_SPEED);
                    break;
            }
            roller.roll(rollerVelocity);
        }
    }

    public boolean neutralHold(){
        return roller.wristIR();
    }

    public void setRollerState(RollerState rollerState) {
        this.rollerState = rollerState;
    }

    

    @Override
    public void simulationPeriodic() {
        roller.updateInputs(rollerIOInputs);
        runRoller();
    }

    @Override
    public void periodic() {
        roller.updateInputs(rollerIOInputs);
        runRoller();
    }
}