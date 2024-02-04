package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIO.ActuatorShoulderIOInputs;

public class ActuatorWristIOSim implements ActuatorWristIO{
    
    private double wristDegrees = 0.0;
    private double wristAngVel = 0.0;
    DCMotorSim wrist = new DCMotorSim(DCMotor.getNeo550(1),5 * 3 * 1.5 * 15/8, 0.025);
    
    @Override
    public void updateInputs(ActuatorWristIOInputs inputs) {
            wrist.update(0.02);
            wristDegrees += wrist.getAngularVelocityRadPerSec() * 0.02;
            wristAngVel = wrist.getAngularVelocityRadPerSec();
            inputs.wristDegrees = wristDegrees;
            inputs.wristAngVel = wristAngVel;
            inputs.wristCurrent = wrist.getCurrentDrawAmps();
            inputs.wristVoltage = 0.0;

    }
    @Override
    public void setVoltage(double voltage){
        wrist.setInputVoltage(voltage);
    }
    @Override
    public void setAngle(double angle){
        wrist.setState(angle, 0.0);
    }
}
