package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShooterIOSim implements ShooterIO {
    FlywheelSim shooterSim = new FlywheelSim(DCMotor.getFalcon500(2), 1.5, 0.001);
    //TODO: Check bounds, gearing, etc
    SingleJointedArmSim anglerSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 150, 0.05, 0.5, 15.0/60, 55.0/60, true, 15.0/60);
    DCMotorSim feederSim = new DCMotorSim(DCMotor.getFalcon500(1), 3, 0.001);

    double shooterVoltage = 0;
    double anglerVoltage = 0;
    double feederVoltage = 0;

    double[] shooterRPS = {0.0, 0.0};
    double theta = 0;
    double thetaRadPerSec = 0;
    
    boolean beamBreakTriggered = false;
    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.shooterRPS = shooterRPS;
        inputs.pivotRad = anglerSim.getAngleRads();
        inputs.pivotRadPerSec = anglerSim.getVelocityRadPerSec();

        inputs.shooterAppliedVolts = new double[]{shooterVoltage, shooterVoltage};
        inputs.shooterAppliedCurrent = new double[]{shooterSim.getCurrentDrawAmps()};

        inputs.pivotAppliedVolts = anglerVoltage;
        inputs.pivotAppliedCurrent = anglerSim.getCurrentDrawAmps();

        inputs.feederAppliedVolts = feederVoltage;
        inputs.feederAppliedCurrent = feederSim.getCurrentDrawAmps();

        inputs.beamBreakTriggered = beamBreakTriggered;
    }
    @Override
    public void setDesiredAngler(double radians, double radiansPerSecond){
        anglerSim.setState(radians, radiansPerSecond);
    }
    @Override
    public void setDesiredRPM(double RPM){
        shooterSim.setState(RPM);
    }   
    @Override
    public void setShooterVoltage(double voltage){
        this.shooterVoltage = voltage;
        shooterSim.setInputVoltage(voltage);
    }
    @Override
    public void setPivotVoltage(double voltage){
        this.anglerVoltage = voltage;
        anglerSim.setInputVoltage(voltage);
    }
    @Override
    public void setFeederVoltage(double voltage){
        this.feederVoltage = voltage;
        feederSim.setInputVoltage(voltage);
    }
    @Override
    public void resetThetaEncoder(){
        anglerSim.setState(0, 0);
    }
    @Override
    public void setBeamBreak(boolean isTriggered){
        beamBreakTriggered = isTriggered;
    }
}
