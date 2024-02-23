package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterAnglerConstants;
import frc.robot.Constants.ShooterFlywheelConstants;

public class ShooterIOSim implements ShooterIO {
    FlywheelSim shooterSim = new FlywheelSim(DCMotor.getFalcon500(2), ShooterFlywheelConstants.kShooterGearing, 0.001);
    //TODO: Check bounds, gearing, etc
    SingleJointedArmSim anglerSim = new SingleJointedArmSim(DCMotor.getFalcon500(1), ShooterAnglerConstants.kMotorGearing , 0.05, 0.5, Math.toRadians(15),Math.toRadians(60) , true, Math.toRadians(15));
    DCMotorSim feederSim = new DCMotorSim(DCMotor.getFalcon500(1),  2.0/1.0, 0.001);

    double shooterVoltage = 0;
    double anglerVoltage = 0;
    double feederVoltage = 0;

    boolean beamBreakTriggered = false;
    @Override
    public void updateInputs(ShooterIOInputs inputs){
        inputs.shooterRPS = new double[]{shooterSim.getAngularVelocityRadPerSec(), shooterSim.getAngularVelocityRadPerSec()};
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
    public void resetPivotEncoder(){
        anglerSim.setState(0, 0);
    }
    @Override
    public void setBeamBreak(boolean isTriggered){
        beamBreakTriggered = isTriggered;
    }
}
