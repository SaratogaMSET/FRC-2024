package frc.robot.subsystems.IntakeSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIOInputsAutoLogged;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIOInputsAutoLogged;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIO.ActuatorWristIOInputs;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.AcutatorConstants;
import frc.robot.Constants.Intake.AcutatorConstants.ActuatorState;
import frc.robot.Constants.Intake.AcutatorConstants.ControlsConstants;

public class IntakeSubsystem extends SubsystemBase {
    public ActuatorShoulderIO shoulder;
    public ActuatorWristIO wrist;
    double wristVelocity = 0;
    double wristAngle = 0;
    double shoulderVelocity = 0;
    double shoulderAngle = 0;
    double shoulderAngVel = 0.0;
    ActuatorState actuatorState;
    ActuatorShoulderIOInputsAutoLogged shoulderIOInputs = new ActuatorShoulderIOInputsAutoLogged();
    ActuatorWristIOInputsAutoLogged wristIOInputs = new ActuatorWristIOInputsAutoLogged();
    IntakeVisualizer viz = new IntakeVisualizer("Intake", null);

    public IntakeSubsystem(ActuatorShoulderIO shoulder, ActuatorWristIO wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    /**
     * Using the current armState, run the arm TODO: Change velocites
     */
    public void runArm (){
         if (actuatorState == null) {
            System.out.println("WARNING: ArmState not set!");
        } else {
            Logger.recordOutput("Arm State", actuatorState.toString());
            switch (actuatorState) {
                case GROUND_DEPLOY:
                    shoulder.setAngle(Intake.AcutatorConstants.SHOULDER_LOW_BOUND, 1);
                    wrist.setAngle(Intake.AcutatorConstants.WRIST_LOW_BOUND, 1);
                    break;
                case AMP:
                    shoulder.setAngle(Intake.AcutatorConstants.AmpScoringPositions.AMP_SHOULDER_ANGLE, 1);
                    wrist.setAngle(Intake.AcutatorConstants.AmpScoringPositions.AMP_WRIST_ANGLE, 1);
                    break;
                case SOURCE:
                    shoulder.setAngle(Intake.AcutatorConstants.SourceScoringPositions.SOURCE_SHOULDER_ANGLE, 1);
                    wrist.setAngle(Intake.AcutatorConstants.SourceScoringPositions.SOURCE_WRIST_ANGLE, 1);
                    break;
                case NEUTRAL:
                    if (shoulderGetDegrees() > Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE) {
                        shoulder.setAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        wrist.setAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    } else {
                        shoulder.setAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.LOWER_MOTION_SHOULDER_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        wrist.setAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    }
                    break;
                case TRAP:
                    shoulder.setAngle(Intake.AcutatorConstants.TrapScoringPositions.TRAP_WRIST_ANGLE, 1);
                    wrist.setAngle(Intake.AcutatorConstants.TrapScoringPositions.TRAP_SHOULDER_ANGLE, 1);
                    break;
                case MANUAL:
                    break;
            }
        }
    }

    // Set the target arm state
    public void setArmState(ActuatorState actuatorState) {
        this.actuatorState = actuatorState;
    }

    public double shoulderGetDegrees(){
        return shoulderIOInputs.shoulderDegrees;
    }
    @Override
    public void simulationPeriodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        runArm();
        viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        runArm();
        viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }
}