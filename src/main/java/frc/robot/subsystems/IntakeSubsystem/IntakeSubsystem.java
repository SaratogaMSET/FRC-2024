package frc.robot.subsystems.IntakeSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIOInputsAutoLogged;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIOInputsAutoLogged;
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
                    setAngleShoulder(Intake.AcutatorConstants.SHOULDER_LOW_BOUND, 1);
                    setAngleWrist(Intake.AcutatorConstants.WRIST_LOW_BOUND, 1);
                    break;
                case AMP:
                    setAngleShoulder(Intake.AcutatorConstants.AmpScoringPositions.AMP_SHOULDER_ANGLE, 1);
                    setAngleWrist(Intake.AcutatorConstants.AmpScoringPositions.AMP_WRIST_ANGLE, 1);
                    break;
                case SOURCE:
                    setAngleShoulder(Intake.AcutatorConstants.SourceScoringPositions.SOURCE_SHOULDER_ANGLE, 1);
                    setAngleWrist(Intake.AcutatorConstants.SourceScoringPositions.SOURCE_WRIST_ANGLE, 1);
                    break;
                case NEUTRAL:
                    if (shoulderGetDegrees() > Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE) {
                        setAngleShoulder(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        setAngleWrist(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    } else {
                        setAngleShoulder(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.LOWER_MOTION_SHOULDER_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        setAngleWrist(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    }
                    break;
                case TRAP:
                    setAngleShoulder(Intake.AcutatorConstants.TrapScoringPositions.TRAP_WRIST_ANGLE, 1);
                    setAngleWrist(Intake.AcutatorConstants.TrapScoringPositions.TRAP_SHOULDER_ANGLE, 1);
                    break;
                case MANUAL:
                    break;
            }
        }
    }

    public void setAngleWrist(double angle, double velocity){
        double wristDegrees = wristGetDegrees();
        if (Math.abs(velocity) > 1)
        velocity = Math.signum(velocity);
        if (velocity < 0)
        velocity = 0;

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle

        angle = Math.min(AcutatorConstants.WRIST_HIGH_BOUND, Math.max(angle, AcutatorConstants.WRIST_LOW_BOUND));

        // Calculate gravity ff + PID
        double error = (angle - wristDegrees) / (AcutatorConstants.WRIST_HIGH_BOUND - AcutatorConstants.WRIST_LOW_BOUND);
        double gravity = ControlsConstants.k_G * Math.cos(wristDegrees + AcutatorConstants.WRIST_ENCODER_OFFSET_FROM_ZERO);

        // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
        if (angle > wristDegrees) {
            wrist.setVoltage((ControlsConstants.k_P * error * power) - gravity);
        } else {
            wrist.setVoltage(((ControlsConstants.k_P * error) * power));
        }
    }
    public void setAngleShoulder(double angle, double velocity){
        double shoulderDegrees = shoulderGetDegrees();
        if (Math.abs(velocity) > 1)
            velocity = Math.signum(velocity);
        if (velocity < 0)
            velocity = 0;

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle

        angle = Math.min(AcutatorConstants.SHOULDER_HIGH_BOUND, Math.max(angle, AcutatorConstants.SHOULDER_LOW_BOUND));

        // Calculate gravity ff + PID
        double error = (angle - shoulderDegrees) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND);
        double gravity = ControlsConstants.k_G * Math.cos(shoulderDegrees + AcutatorConstants.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

        // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
        if (angle > shoulderDegrees) {
            shoulder.setVoltage((ControlsConstants.k_P * error * power) - gravity);
        } else {
            shoulder.setVoltage(((ControlsConstants.k_P * error) * power));
        }
    }
    // Set the target arm state
    public void setArmState(ActuatorState actuatorState) {
        this.actuatorState = actuatorState;
    }

    public double shoulderGetDegrees(){
        return shoulderIOInputs.shoulderDegrees;
    }
    public double wristGetDegrees(){
        return wristIOInputs.wristDegrees;
    }
    @Override
    public void simulationPeriodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        // runArm();
        viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        // runArm();
        viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }
}