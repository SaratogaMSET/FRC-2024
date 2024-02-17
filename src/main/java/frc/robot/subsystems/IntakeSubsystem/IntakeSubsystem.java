package frc.robot.subsystems.IntakeSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIOInputsAutoLogged;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIOInputsAutoLogged;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIOReal;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.AcutatorConstants;
import frc.robot.Constants.Intake.AcutatorConstants.ActuatorState;
import frc.robot.Constants.Intake.AcutatorConstants.WristControlsConstants;
import frc.robot.Constants.Intake.AcutatorConstants.ShoulderControlsConstants;


public class IntakeSubsystem extends SubsystemBase {
    public ActuatorShoulderIO shoulder;
    public ActuatorWristIO wrist;
    double wristVelocity = 0;
    double wristAngle = 0;
    double shoulderVelocity = 0;
    double shoulderAngle = 0;
    double shoulderAngVel = 0.0;
    boolean previousHallEffect = false;
    ActuatorState actuatorState = ActuatorState.NEUTRAL;
    ActuatorShoulderIOInputsAutoLogged shoulderIOInputs = new ActuatorShoulderIOInputsAutoLogged();
    ActuatorWristIOInputsAutoLogged wristIOInputs = new ActuatorWristIOInputsAutoLogged();
    IntakeVisualizer viz = new IntakeVisualizer("Intake", null);
    PIDController wristPID = new PIDController(WristControlsConstants.k_P, WristControlsConstants.k_I, WristControlsConstants.k_D);
    PIDController shoulderPID = new PIDController(ShoulderControlsConstants.k_P, ShoulderControlsConstants.k_I, ShoulderControlsConstants.k_D);

    public IntakeSubsystem(ActuatorShoulderIO shoulder, ActuatorWristIO wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    /**
     * Using the current armState, run the arm TODO: Change velocites
     */
    public void runArm(){
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
                setAngleShoulder(Intake.AcutatorConstants.TrapScoringPositions.TRAP_SHOULDER_ANGLE, 1);
                setAngleWrist(Intake.AcutatorConstants.TrapScoringPositions.TRAP_WRIST_ANGLE, 1); 
                break;
            case MANUAL:
                break;
        }
    }

    public void setAngleWrist(double angle, double velocity){
        double wristDegrees = wristGetDegrees();

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle      
        angle = MathUtil.clamp(angle, AcutatorConstants.WRIST_LOW_BOUND, AcutatorConstants.WRIST_HIGH_BOUND);
        // angle = Math.min(AcutatorConstants.WRIST_HIGH_BOUND, Math.max(angle, AcutatorConstants.WRIST_LOW_BOUND));

        // Calculate gravity ff + PID
        double pidOutput = wristPID.calculate(wristDegrees, angle) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND) * power;
        // double error = (angle - wristDegrees) / (AcutatorConstants.WRIST_HIGH_BOUND - AcutatorConstants.WRIST_LOW_BOUND);
        double gravity = WristControlsConstants.k_G * Math.cos(wristDegrees + AcutatorConstants.WRIST_ENCODER_OFFSET_FROM_ZERO);

        // Move to target
        wrist.setVoltage(pidOutput - gravity);

        Logger.recordOutput("Arm/Wrist/Angle Setpoint", angle);
        Logger.recordOutput("Arm/Wrist/Current Angle", wristDegrees);
    }

    public void setAngleShoulder(double angle, double velocity){
        double shoulderDegrees = shoulderGetDegrees();

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle

        angle = MathUtil.clamp(angle, AcutatorConstants.SHOULDER_LOW_BOUND, AcutatorConstants.SHOULDER_HIGH_BOUND);

        // Calculate gravity ff + PID
        double pidOutput = shoulderPID.calculate(shoulderDegrees, angle) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND) * power;
        // double error = (angle - shoulderDegrees) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND);
        double gravity = ShoulderControlsConstants.k_G * Math.cos(Math.toRadians(shoulderDegrees + AcutatorConstants.SHOULDER_ENCODER_OFFSET_FROM_ZERO));

        // Move to target
        shoulder.setVoltage(pidOutput - gravity);

        Logger.recordOutput("Arm/Shoulder/Angle Setpoint", angle);
        Logger.recordOutput("Arm/Shoulder/Current Angle", shoulderDegrees);
    }

    public void hallEffect(){
        if(!previousHallEffect && wristIOInputs.hallEffects){
            wrist.setAngle(wristAngle, 0);
            wristIOInputs.wristDegrees = AcutatorConstants.WRIST_ENCODER_HALL_EFFECT;
        }
        previousHallEffect = wristIOInputs.hallEffects;
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
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        // runArm();
        viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        hallEffect();
        // runArm();
        viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }
}