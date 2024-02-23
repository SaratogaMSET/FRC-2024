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
            case GROUND_DEPLOY:
                setAngleShoulder(Intake.Shoulder.LOW_BOUND, 0.03);
                setAngleWrist(Intake.Wrist.LOW_BOUND, 0.03);
                break;
            case AMP:
                setAngleShoulder(Intake.DesiredStates.Amp.SHOULDER_ANGLE, 0.03);
                setAngleWrist(Intake.DesiredStates.Amp.WRIST_ANGLE, 0.03);
                break;
            case SOURCE:
                setAngleShoulder(Intake.DesiredStates.Source.SHOULDER_ANGLE, 0.1);
                setAngleWrist(Intake.DesiredStates.Source.WRIST_ANGLE, 0.1);
                break;
            case NEUTRAL:
                if (shoulderGetDegrees() > Intake.DesiredStates.Ground.UPPER_MOTION_SHOULDER_ANGLE) {
                    setAngleShoulder(Intake.DesiredStates.Ground.UPPER_MOTION_SHOULDER_ANGLE,
                            0.1);
                    setAngleWrist(Intake.DesiredStates.Ground.UPPER_MOTION_WRIST_ANGLE,
                             0.1);
                } else {
                    setAngleShoulder(Intake.DesiredStates.Ground.LOWER_MOTION_SHOULDER_ANGLE,
                            Intake.DesiredStates.Ground.SHOULDER_POWER_PERCENT);
                    setAngleWrist(Intake.DesiredStates.Ground.LOWER_MOTION_WRIST_ANGLE,
                            Intake.DesiredStates.Ground.WRIST_POWER_PERCENT);
                }
                break;
            case TRAP:
                setAngleShoulder(Intake.DesiredStates.Trap.SHOULDER_ANGLE, 0.03);
                setAngleWrist(Intake.DesiredStates.Trap.WRIST_ANGLE, 0.03); 
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
        double wristDegrees = wristGetDegrees();

        // Calculate the voltage draw 
        double power = 12 * Math.abs(velocity);

        // Enforce bounds on angle      
        angle = MathUtil.clamp(angle, Wrist.LOW_BOUND, Wrist.HIGH_BOUND);
        // angle = Math.min(AcutatorConstants.WRIST_HIGH_BOUND, Math.max(angle, AcutatorConstants.WRIST_LOW_BOUND));

        // enable movement towards least movement
        // angle = Math.abs(angle  - wristDegrees) > 180 ? angle - 360 : angle;

        // Calculate gravity ff + PID
        double pidOutput = wristPID.calculate(wristDegrees, angle) / (Wrist.HIGH_BOUND - Wrist.LOW_BOUND) * power;
        // double error = (angle - wristDegrees) / (AcutatorConstants.WRIST_HIGH_BOUND - AcutatorConstants.WRIST_LOW_BOUND);
        double gravity = Wrist.ControlsConstants.k_G * Math.cos(wristDegrees + Wrist.ENCODER_OFFSET_FROM_ZERO);

        // Move to target
        wrist.setVoltage(pidOutput - gravity);

        Logger.recordOutput("Arm/Wrist/Angle Setpoint", angle);
        Logger.recordOutput("Arm/Wrist/Current Angle", wristDegrees);
        SmartDashboard.putNumber("Angle voltage output", pidOutput - gravity);
        SmartDashboard.putNumber("Angle voltage PID OUTPUT", pidOutput);
        SmartDashboard.putNumber("Angle voltage GRAVITY OUTPUT", gravity);
        SmartDashboard.putNumber("Power", power);
        SmartDashboard.putNumber("Angle set point", angle);
        SmartDashboard.putNumber("Current angle", wristDegrees);
        SmartDashboard.putNumber("wristPID error", wristPID.getPositionError());
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

    /** Resets wrist motor encoder if the wrist has just reached close to the sensor*/
    private void hallEffect(){
        if(!previousHallEffect && wristIOInputs.hallEffects){
            wrist.setAngle(wristAngle, 0);
            // wristIOInputs.wristDegrees = AcutatorConstants.WRIST_ENCODER_HALL_EFFECT;  TODO: WHAT is this code supposed to do? 
        }
        previousHallEffect = wristIOInputs.hallEffects;
    }

    // Set the target arm state
    public void setArmState(ArmStates armState) {
        this.armState = armState;
    }

    /**Returns current shoulder position in degrees
     * @returns shoulderDegrees
    */
    public double shoulderGetDegrees(){
        return shoulderIOInputs.shoulderDegrees;
    }

    /**Returns current wrist position in degrees
     * @returns wristDegrees
    */
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
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        hallEffect();
        // runArm();
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }
}