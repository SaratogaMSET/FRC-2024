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
import frc.robot.subsystems.Intake.Shoulder.ShoulderIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Wrist.WristIOInputsAutoLogged;
import frc.robot.subsystems.Intake.RollerSubsystem.RollerIO;
import frc.robot.subsystems.Intake.RollerSubsystem.RollerIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIO;
import frc.robot.subsystems.Intake.Wrist.WristIO;


public class IntakeSubsystem extends SubsystemBase {
    public ShoulderIO shoulder;
    public WristIO wrist;
    public RollerIO roller;
    ShoulderIOInputsAutoLogged shoulderIOInputs = new ShoulderIOInputsAutoLogged();
    WristIOInputsAutoLogged wristIOInputs = new WristIOInputsAutoLogged();
    RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

    boolean previousHallEffect = false;
    ArmStates armState = ArmStates.NEUTRAL;

    public IntakeSubsystem(ShoulderIO shoulder, WristIO wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    public double shoulderGetRads(){
        return shoulderIOInputs.shoulderRads;
    }
    public double shoulderGetRadPerSec(){
        return shoulderIOInputs.shoulderRadPerSecs;
    }
    public void setShoulderVoltage(double voltage){
        shoulder.setVoltage(voltage); //TODO: Enforce bound here
        Logger.recordOutput("Intake/Shoulder/Voltage", voltage);
    }
    public double wristGetRads(){
        return wristIOInputs.rads;
    }
    public double wristGetRadPerSec(){
        return wristIOInputs.radsPerSec;
    }
    public void setWristVoltage(double voltage){
        wrist.setVoltage(voltage); //TODO: Enforce bound here
        Logger.recordOutput("Intake/Wrist/Voltage", voltage);
    }
    public void setRollerVoltage(double voltage){
        roller.setVoltage(voltage);
        Logger.recordOutput("Intake/Roller/Voltage", voltage);
    }
    private void setAngleWrist(double angle){
        // Enforce bounds on angle      
        angle = MathUtil.clamp(angle, Wrist.LOW_BOUND, Wrist.HIGH_BOUND);

        double voltageFB = (angle - wristGetRads()) * Wrist.k_P + wristGetRadPerSec() * Wrist.k_D;
        double voltageFF = Math.cos(wristGetRads()) * Wrist.k_G;

        setWristVoltage(voltageFB + voltageFF);

        Logger.recordOutput("Intake/Wrist/Angle Setpoint", angle);
        Logger.recordOutput("Intake/Wrist/Current Angle", wristGetRads());
    }

    /**Sets wrist angle (between the given bounds) using the PID and outputs calculated values
     * @param angle target angle measure
     * @param velocity speed of the shoudler
    */
    private void setAngleShoulder(double angle){
        double shoulderRads = shoulderGetRads();

        // Enforce bounds on angle
        angle = MathUtil.clamp(angle, Shoulder.LOW_BOUND, Shoulder.HIGH_BOUND);


        double voltageFB = (angle - wristGetRads()) * Shoulder.k_P + wristGetRadPerSec() * Shoulder.k_D;
        double voltageFF = Math.cos(wristGetRads()) * Shoulder.k_G;

        setShoulderVoltage(voltageFB + voltageFF);

        Logger.recordOutput("Intake/Shoulder/Setpoint", angle);
        Logger.recordOutput("Intake/Shoulder/Angle", shoulderRads);
    }

    // Set the target arm state
    public void setArmState(ArmStates armState) {
        this.armState = armState;
    }

    @Override
    public void simulationPeriodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        roller.updateInputs(rollerIOInputs);
        wrist.hallEffectReset();
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        Logger.processInputs(getName(), rollerIOInputs);
        // runArm();
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        roller.updateInputs(rollerIOInputs);
        wrist.hallEffectReset();
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        Logger.processInputs(getName(), rollerIOInputs);
        // runArm();
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    }
}