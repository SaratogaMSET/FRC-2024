package frc.robot.subsystems.Intake.RollerSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.Roller;
import frc.robot.Constants.Intake.Roller.RollerState;
import frc.robot.subsystems.Intake.RollerSubsystem.RollerSubsystemIO.RollerSubsystemIOInputs;

public class RollerSubsystem extends SubsystemBase {
    RollerSubsystemIO roller;
    double rollerVelocity = 0.0;
    RollerState rollerState = RollerState.NEUTRAL;
    RollerSubsystemIOInputs rollerIOInputs = new RollerSubsystemIOInputs();

    public RollerSubsystem(RollerSubsystemIO roller) {
        this.roller = roller;
    }

    /**
     * Using the current rollerState (intake, outtake, nuetral, hold), runs the roller
     */
    private void runRoller (){
        //  if (rollerState == null) {
        //     System.out.println("WARNING: RollerState not set!");
        // } 
        // else {
            switch (rollerState) {
                case INTAKE:
                    roller.roll(Roller.ROLLING_SPEED);
                    break;
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
        }
    // }

     /**
     * @return Retruns the shooter IR gate state
     */
    public boolean neutralHold(){
        return rollerIOInputs.shooterIR;
    }

    /**
     * @return Retruns roller state (intake, outtake, nuetral, hold)
     */
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