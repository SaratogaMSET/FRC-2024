package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.Constants.Intake.Shoulder;
import frc.robot.Constants.Intake.Wrist;
import frc.robot.Constants.Intake.DesiredStates.ArmStates;
import frc.robot.subsystems.Intake.ActuatorShoulder.ActuatorShoulderIO;
import frc.robot.subsystems.Intake.ActuatorShoulder.ActuatorShoulderIOInputsAutoLogged;
import frc.robot.subsystems.Intake.ActuatorWrist.ActuatorWristIO;
import frc.robot.subsystems.Intake.ActuatorWrist.ActuatorWristIOInputsAutoLogged;


public class IntakeSubsystem extends SubsystemBase {
    public ActuatorShoulderIO shoulder;
    public ActuatorWristIO wrist;
    double wristVelocity = 0;
    double wristAngle = 0;
    double shoulderVelocity = 0;
    double shoulderAngle = 0;
    double shoulderAngVel = 0.0;
    boolean previousHallEffect = false;
    ArmStates armState = ArmStates.NEUTRAL;
    ActuatorShoulderIOInputsAutoLogged shoulderIOInputs = new ActuatorShoulderIOInputsAutoLogged();
    ActuatorWristIOInputsAutoLogged wristIOInputs = new ActuatorWristIOInputsAutoLogged();
    // IntakeVisualizer viz = new IntakeVisualizer("Intake", null);
    PIDController wristPID = new PIDController(Wrist.ControlsConstants.k_P, Wrist.ControlsConstants.k_I, Wrist.ControlsConstants.k_D);
    PIDController shoulderPID = new PIDController(Shoulder.ControlsConstants.k_P, Shoulder.ControlsConstants.k_I, Shoulder.ControlsConstants.k_D);

    public IntakeSubsystem(ActuatorShoulderIO shoulder, ActuatorWristIO wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    /**
     * Using the current armState, runs the arm TODO: Change velocites
     * States: Amp, Ground Deploy, SOurce, Neutral, Trap or Mannual (does nothing)
     */
    public void runArm(){
        Logger.recordOutput("Arm State", armState.toString());
        switch (armState) {
            case NEUTRAL:
                setAngleShoulder(Intake.DesiredStates.Neutral.SHOULDER_ANGLE, Intake.DesiredStates.Neutral.SHOULDER_VELOCITY);
                setAngleWrist(Intake.DesiredStates.Neutral.WRIST_ANGLE, Intake.DesiredStates.Neutral.WRIST_VELOCITY);
                break;
            case AMP:
                setAngleShoulder(Intake.DesiredStates.Amp.SHOULDER_ANGLE, Intake.DesiredStates.Amp.SHOULDER_VELOCITY);
                setAngleWrist(Intake.DesiredStates.Amp.WRIST_ANGLE, Intake.DesiredStates.Amp.WRIST_VELOCITY);
                break;
            case SOURCE:
                setAngleShoulder(Intake.DesiredStates.Source.SHOULDER_ANGLE, Intake.DesiredStates.Source.SHOULDER_VELOCITY);
                setAngleWrist(Intake.DesiredStates.Source.WRIST_ANGLE, Intake.DesiredStates.Source.WRIST_VELOCITY);
                break;
            case GROUND_DEPLOY:
                if (shoulderGetDegrees() > Intake.DesiredStates.Ground.UPPER_MOTION_SHOULDER_ANGLE) {
                    setAngleShoulder(Intake.DesiredStates.Ground.UPPER_MOTION_SHOULDER_ANGLE,
                            1);
                    setAngleWrist(Intake.DesiredStates.Ground.UPPER_MOTION_WRIST_ANGLE,
                             Intake.DesiredStates.Ground.WRIST_VELOCITY);
                } else {
                    setAngleShoulder(Intake.DesiredStates.Ground.LOWER_MOTION_SHOULDER_ANGLE,
                            1);
                    setAngleWrist(Intake.DesiredStates.Ground.LOWER_MOTION_WRIST_ANGLE,
                            Intake.DesiredStates.Ground.WRIST_VELOCITY);
                }
                break;
            case TRAP:
                setAngleShoulder(Intake.DesiredStates.Trap.SHOULDER_ANGLE, Intake.DesiredStates.Trap.SHOULDER_VELOCITY);
                setAngleWrist(Intake.DesiredStates.Trap.WRIST_ANGLE, Intake.DesiredStates.Trap.WRIST_VELOCITY); 
                break;
            case MANUAL:
                break;
        }
    }

    /**Sets wrist angle (between the given bounds) using the PID and outputs calculated values
     * @param angle target angle measure
     * @param velocity speed of the wrist
    */
    private void setAngleWrist(double angle, double velocity){
        double wristDegrees = wristGetDegrees() + shoulderIOInputs.shoulderDegrees;

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle      
        angle = MathUtil.clamp(angle, Wrist.LOW_BOUND, Wrist.HIGH_BOUND);
        // angle = Math.min(AcutatorConstants.WRIST_HIGH_BOUND, Math.max(angle, AcutatorConstants.WRIST_LOW_BOUND));

        // Calculate gravity ff + PID
        double pidOutput = wristPID.calculate(wristDegrees, angle) / (Wrist.HIGH_BOUND - Wrist.LOW_BOUND) * power;
        // double error = (angle - wristDegrees) / (AcutatorConstants.WRIST_HIGH_BOUND - AcutatorConstants.WRIST_LOW_BOUND);
        double gravity = Wrist.ControlsConstants.k_G * Math.cos(wristDegrees + Wrist.ENCODER_OFFSET_FROM_ZERO);

        // Move to target
        wrist.setVoltage(pidOutput - gravity);

        Logger.recordOutput("Arm/Wrist/Angle Setpoint", angle);
        Logger.recordOutput("Arm/Wrist/Current Angle", wristDegrees);
        Logger.recordOutput("Arm/Wrist/Voltage", pidOutput - gravity);
    }

    /**Sets wrist angle (between the given bounds) using the PID and outputs calculated values
     * @param angle target angle measure
     * @param velocity speed of the shoudler
    */
    private void setAngleShoulder(double angle, double velocity){
        double shoulderDegrees = shoulderGetDegrees();

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle

        angle = MathUtil.clamp(angle, Shoulder.LOW_BOUND, Shoulder.HIGH_BOUND);

        // Calculate gravity ff + PID
        double pidOutput = shoulderPID.calculate(shoulderDegrees, angle) / (Shoulder.HIGH_BOUND - Shoulder.LOW_BOUND) * power;
        // double error = (angle - shoulderDegrees) / (AcutatorConstants.SHOULDER_HIGH_BOUND - AcutatorConstants.SHOULDER_LOW_BOUND);
        double gravity = Shoulder.ControlsConstants.k_G * Math.cos(Math.toRadians(shoulderDegrees + Shoulder.ENCODER_OFFSET_FROM_ZERO));

        // Move to target
        shoulder.setVoltage(pidOutput - gravity);

        Logger.recordOutput("Arm/Shoulder/Angle Setpoint", angle);
        Logger.recordOutput("Arm/Shoulder/Current Angle", shoulderDegrees);
    }

    // Set the target arm state
    public void setArmState(ArmStates armState) {
        this.armState = armState;
    }

    /**Returns current shoulder position in degrees, 
     * @returns shoulderDegrees
    */
    public double shoulderGetDegrees(){
        return shoulderIOInputs.shoulderDegrees;
    }

    /**Returns current wrist position in degrees
     * @returns wristDegrees
    */
    public double wristGetDegrees(){
        return wristIOInputs.wristDegrees - shoulderIOInputs.shoulderDegrees;
    }

    @Override
    public void simulationPeriodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        wrist.hallEffectReset();
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        // runArm();
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        wrist.hallEffectReset();
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        // runArm();
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }
}