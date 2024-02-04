package frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ActuatorShoulderIOSim implements ActuatorShoulderIO{

    private double shoulderDegrees = 0.0;
    private double shoulderAngVel = 0.0;
    DCMotorSim shoulder = new DCMotorSim(DCMotor.getFalcon500(1),5 * 3 * 1.5 * 15/8, 0.025);

    
    @Override
    public void updateInputs(ActuatorShoulderIOInputs inputs) {
            shoulder.update(0.02);
            shoulderDegrees += shoulder.getAngularVelocityRadPerSec() * 0.02;
            shoulderAngVel = shoulder.getAngularVelocityRadPerSec();
            inputs.shoulderDegrees = shoulderDegrees; 
            inputs.shoulderAngVel = shoulderAngVel; 
            inputs.shoulderCurrent = shoulder.getCurrentDrawAmps();
            inputs.shoulderVoltage = 0.0;
    }
    @Override
    public void setVoltage(double voltage){
        shoulder.setInputVoltage(voltage);
    }
    @Override
    public void setAngle(double angle){
        shoulder.setState(angle, 0.0);
    }
}
