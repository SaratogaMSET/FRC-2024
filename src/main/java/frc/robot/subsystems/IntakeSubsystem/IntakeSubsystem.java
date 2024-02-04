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
    ActuatorShoulderIO shoulder;
    ActuatorWristIO wrist;
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
                    shoulderSetAngle(Intake.AcutatorConstants.SHOULDER_LOW_BOUND, 1);
                    wristSetAngle(Intake.AcutatorConstants.WRIST_LOW_BOUND, 1);
                    break;
                case AMP:
                    shoulderSetAngle(Intake.AcutatorConstants.AmpScoringPositions.AMP_SHOULDER_ANGLE, 1);
                    wristSetAngle(Intake.AcutatorConstants.AmpScoringPositions.AMP_WRIST_ANGLE, 1);
                    break;
                case SOURCE:
                    shoulderSetAngle(Intake.AcutatorConstants.SourceScoringPositions.SOURCE_WRIST_ANGLE, 1);
                    wristSetAngle(Intake.AcutatorConstants.SourceScoringPositions.SOURCE_SHOULDER_ANGLE, 1);
                    break;
                case NEUTRAL:
                    if (shoulderGetDegrees() > Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE) {
                        shoulderSetAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        wristSetAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    } else {
                        shoulderSetAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.LOWER_MOTION_SHOULDER_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        wristSetAngle(Intake.AcutatorConstants.GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE,
                                Intake.AcutatorConstants.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    }
                    break;
                case TRAP:
                    shoulderSetAngle(Intake.AcutatorConstants.TrapScoringPositions.TRAP_WRIST_ANGLE, 1);
                    wristSetAngle(Intake.AcutatorConstants.TrapScoringPositions.TRAP_SHOULDER_ANGLE, 1);
                    break;
                case MANUAL:
                    break;
            }
        }
    }

    public void shoulderSetAngle(double angle, double velocity){
        shoulderAngle = angle;
        if (Math.abs(velocity) > 1)
        velocity = Math.signum(velocity);
    if (velocity < 0)
        velocity = 0;

    // Calculate the voltage draw 
    double power = 12 * Math.abs(velocity);

    // Enforce bounds on angle

    angle = Math.min(AcutatorConstants.SHOULDER_HIGH_BOUND, Math.max(angle, AcutatorConstants.SHOULDER_LOW_BOUND));

    // Calculate gravity ff + PID
    double error = (angle - shoulderIOInputs.shoulderDegrees) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND);
    double gravity = ControlsConstants.k_G * Math.cos(shoulderIOInputs.shoulderDegrees + AcutatorConstants.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

    // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
    if (angle > shoulderIOInputs.shoulderDegrees) {
        shoulder.setVoltage((ControlsConstants.k_P * error * power) - gravity);
    } else {
        shoulder.setVoltage(((ControlsConstants.k_P * error) * power));
    }
    }

    public void wristSetAngle(double angle, double velocity){
    wristAngle = angle;
        if (Math.abs(velocity) > 1)
        velocity = Math.signum(velocity);
    if (velocity < 0)
        velocity = 0;

    // Calculate the voltage draw 
    double power = 12 * Math.abs(velocity);

    // Enforce bounds on angle

    angle = Math.min(AcutatorConstants.SHOULDER_HIGH_BOUND, Math.max(angle, AcutatorConstants.SHOULDER_LOW_BOUND));

    // Calculate gravity ff + PID
    double error = (angle - shoulderIOInputs.shoulderDegrees) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND);
    double gravity = ControlsConstants.k_G * Math.cos(shoulderIOInputs.shoulderDegrees + AcutatorConstants.SHOULDER_ENCODER_OFFSET_FROM_ZERO);

    // If the target is to move upward, then use gravity ff + PID. Otheriwse, use only PID
    if (angle > shoulderIOInputs.shoulderDegrees) {
        wrist.setVoltage((ControlsConstants.k_P * error * power) - gravity);
    } else {
        wrist.setVoltage(((ControlsConstants.k_P * error) * power));
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
        viz.updateSim(shoulderAngle, wristAngle);
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
        runArm();
    }
}