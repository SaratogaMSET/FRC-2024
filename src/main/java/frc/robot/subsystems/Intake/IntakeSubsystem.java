package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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
    // public CANSparkMax WristMotorSus = new CANSparkMax(Wrist.MOTOR, MotorType.kBrushless);
    public WristIO wrist;
    // public RollerIO roller;
    ShoulderIOInputsAutoLogged shoulderIOInputs = new ShoulderIOInputsAutoLogged();
    WristIOInputsAutoLogged wristIOInputs = new WristIOInputsAutoLogged();

    PIDController wristPID = new PIDController(Wrist.k_P, 0.0, Wrist.k_D);
    PIDController shoulderPID = new PIDController(Shoulder.k_P, 0.0, Shoulder.k_D);
    boolean previousHallEffect = false;

    public IntakeSubsystem(ShoulderIO shoulder, WristIO wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
        if(wrist.getHallEffect()) wrist.manualHallEffectReset();
        // this.roller = roller;
    }

    public double shoulderGetRads(){
        return shoulderIOInputs.shoulderRads;
    }
    public double shoulderGetRadPerSec(){
        return shoulderIOInputs.shoulderRadPerSecs;
    }
    public void setShoulderVoltage(double voltage){
        if(voltage < 0 && shoulderGetRads() * 180/Math.PI < -30) voltage = 0;
        if(voltage > 0 && shoulderGetRads() * 180/Math.PI > 90) voltage = 0;
        shoulder.setVoltage(voltage);
        Logger.recordOutput("Intake/Shoulder/Voltage", voltage);
    }
    public double wristGetRads(){
        return wristIOInputs.wristRads;
    }
    public double wristGetRadPerSec(){
        return wristIOInputs.wristRadsPerSec;
    }
    public boolean getHallEffect(){
        return wrist.getHallEffect();
    }
    public void setWristVoltage(double voltage){
        // if(wristGetRads() > Wrist.HIGH_BOUND && voltage > 0) voltage = 0;
        // if(wrist.getHallEffect() && voltage < 0) voltage = 0;
        
        wrist.motor.setVoltage(voltage);
        Logger.recordOutput("Intake/Wrist/Voltage", voltage);
    }
    public void setVoltages(double shoulderVoltage, double wristVoltage){
        setShoulderVoltage(shoulderVoltage);
        setWristVoltage(wristVoltage);
    }
    // public void setRollerVoltage(double voltage){
    //     roller.setVoltage(voltage);
    //     Logger.recordOutput("Intake/Roller/Voltage", voltage);
    // }
    public Command setGravityCompensation(double additionalVoltage){
        double voltageFF = Math.cos(shoulderGetRads() - 0.14) * Shoulder.k_G + additionalVoltage;
        return this.run(()->setShoulderVoltage(voltageFF));
    }
    public void setAngleWrist(double angle){
        // Enforce bounds on angle      
        // angle = MathUtil.clamp(angle, Wrist.LOW_BOUND, Wrist.HIGH_BOUND);

        double voltageFB = wristPID.calculate(wristGetRads(), angle);

        setWristVoltage(voltageFB);

        Logger.recordOutput("Intake/Wrist/Angle Setpoint", angle);
        Logger.recordOutput("Intake/Wrist/Current Angle", wristGetRads());
    }
    public void setAngleShoulder(double angle){
        double shoulderRads = shoulderGetRads();

        // angle = MathUtil.clamp(angle, Shoulder.HIGH_BOUND, Shoulder.LOW_BOUND);
        // Enforce bounds on angle
        angle = MathUtil.clamp(angle, Shoulder.LOW_BOUND, Shoulder.HIGH_BOUND);

        double voltageFB = MathUtil.clamp(shoulderPID.calculate(shoulderRads, angle), -4, 4);
        // double maxVoltFromVelocity = shoulderGetRadPerSec() * 1.0 + 0.5;
        // if(Math.abs(voltageFB) > maxVoltFromVelocity) voltageFB = Math.signum(voltageFB) * maxVoltFromVelocity; 
        double voltageFF = Math.cos(shoulderRads - 0.14) * Shoulder.k_G;
        setShoulderVoltage(voltageFB + voltageFF);

        Logger.recordOutput("Intake/Shoulder/Setpoint", angle);
    }
    @Override
    public void simulationPeriodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        // Logger.processInputs(getName(), rollerIOInputs);
    }
    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        // roller.updateInputs(rollerIOInputs);
        // wrist.hallEffectReset();
        Logger.recordOutput("Intake/Shoulder/Angle", shoulderGetRads() * 180 / Math.PI);
        Logger.recordOutput("Intake/Shoulder/FF", Math.cos(shoulderGetRads()) * Shoulder.k_G);

        Logger.processInputs(getName(), shoulderIOInputs);
        Logger.processInputs(getName(), wristIOInputs);
        // Logger.processInputs(getName(), rollerIOInputs);
        // runArm();
        // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
    
        // Logger.processInputs(getName(), shoulderIOInputs);
        // Logger.processInputs(getName(), wristIOInputs);
        // Logger.processInputs(getName(), rollerIOInputs);
    }
}