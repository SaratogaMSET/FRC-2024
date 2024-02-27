package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.Shoulder;
import frc.robot.Constants.Intake.Wrist;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Wrist.WristIOInputsAutoLogged;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOInputsAutoLogged;
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

    public IntakeSubsystem(ShoulderIO shoulder, WristIO wrist, RollerIO roller) {
        this.shoulder = shoulder;
        this.wrist = wrist;
        this.roller = roller;
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
    public void setAngleWrist(double angle){
        // Enforce bounds on angle      
        angle = MathUtil.clamp(angle, Wrist.LOW_BOUND, Wrist.HIGH_BOUND);

        double voltageFB = (angle - wristGetRads()) * Wrist.k_P + wristGetRadPerSec() * Wrist.k_D;
        double voltageFF = Math.cos(wristGetRads()) * Wrist.k_G;

        setWristVoltage(voltageFB + voltageFF);

        Logger.recordOutput("Intake/Wrist/Angle Setpoint", angle);
        Logger.recordOutput("Intake/Wrist/Current Angle", wristGetRads());
    }
    public void setAngleShoulder(double angle){
        double shoulderRads = shoulderGetRads();

        // Enforce bounds on angle
        angle = MathUtil.clamp(angle, Shoulder.LOW_BOUND, Shoulder.HIGH_BOUND);


        double voltageFB = (angle - wristGetRads()) * Shoulder.k_P + wristGetRadPerSec() * Shoulder.k_D;
        double voltageFF = Math.cos(wristGetRads()) * Shoulder.k_G;

        setShoulderVoltage(voltageFB + voltageFF);

        Logger.recordOutput("Intake/Shoulder/Setpoint", angle);
        Logger.recordOutput("Intake/Shoulder/Angle", shoulderRads);
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