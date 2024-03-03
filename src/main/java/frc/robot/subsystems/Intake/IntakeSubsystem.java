package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    PIDController wristPID = new PIDController(Wrist.k_P, 0.0, Wrist.k_D);
    PIDController shoulderPID = new PIDController(Shoulder.k_P, 0.0, Shoulder.k_D);
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
        return wristIOInputs.wristRads;
    }
    public double wristGetRadPerSec(){
        return wristIOInputs.wristRadsPerSec;
    }
    public void setWristVoltage(double voltage){
        wrist.setVoltage(voltage); //TODO: Enforce bound here
        Logger.recordOutput("Intake/Wrist/Voltage", voltage);
    }
    public void setRollerVoltage(double voltage){
        roller.setVoltage(voltage);
        Logger.recordOutput("Intake/Roller/Voltage", voltage);
    }
    public Command setGravityCompensation(){
        double voltageFF = Math.cos(wristGetRads()) * Shoulder.k_G;
        Logger.recordOutput("Intake/Voltage", voltageFF);
        return this.run(()->setAngleShoulder(voltageFF));
    }
    public void setAngleWrist(double angle){
        // Enforce bounds on angle      
        angle = MathUtil.clamp(angle, Wrist.LOW_BOUND, Wrist.HIGH_BOUND);
        double voltageFB = wristPID.calculate(wristGetRads(), angle);
        double voltageFF = Math.cos(wristGetRads()) * Wrist.k_G;

        setWristVoltage(voltageFB + voltageFF);

        Logger.recordOutput("Intake/Wrist/Angle Setpoint", angle);
        Logger.recordOutput("Intake/Wrist/Current Angle", wristGetRads());
    }
    public void setAngleShoulder(double angle){
        double shoulderRads = shoulderGetRads();

        // angle = MathUtil.clamp(angle, Shoulder.HIGH_BOUND, Shoulder.LOW_BOUND);
        // Enforce bounds on angle
        angle = MathUtil.clamp(angle, Shoulder.LOW_BOUND, Shoulder.HIGH_BOUND);

        double voltageFB = shoulderPID.calculate(shoulderRads, angle);
        double voltageFF = Math.cos(shoulderRads) * Shoulder.k_G;

        setShoulderVoltage(voltageFB + voltageFF);

        Logger.recordOutput("Intake/Shoulder/Setpoint", angle);
        Logger.recordOutput("Intake/Shoulder/Angle", shoulderRads);
    }

    public boolean getBeamBreak(){
        return rollerIOInputs.rollerIR;
    }
    @Override
    public void simulationPeriodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        roller.updateInputs(rollerIOInputs);
        wrist.hallEffectReset();
        // runArm();
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        Logger.processInputs(getName(), rollerIOInputs);
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
    
        // Logger.processInputs(getName(), shoulderIOInputs);
        // Logger.processInputs(getName(), wristIOInputs);
        // Logger.processInputs(getName(), rollerIOInputs);
    }
}