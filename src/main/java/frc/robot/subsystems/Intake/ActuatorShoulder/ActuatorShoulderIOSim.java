package frc.robot.subsystems.Intake.ActuatorShoulder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.Constants.Intake.Shoulder;

public class ActuatorShoulderIOSim implements ActuatorShoulderIO{

    private double shoulderDegrees = 0.0;
    private double shoulderAngVel = 0.0;
    private double inputVoltage = 0.0;
    SingleJointedArmSim shoulder = new SingleJointedArmSim(DCMotor.getFalcon500(1), Shoulder.GEAR_RATIO, Shoulder.MOI,
    Shoulder.ARM_LENGTH, Math.toRadians(Shoulder.LOW_BOUND),
        Math.toRadians(Shoulder.HIGH_BOUND), true, Math.toRadians(DesiredStates.Neutral.SHOULDER_ANGLE));

    
    @Override
    /**Updates inputs for shoulder voltage, current and angle in degrees and angleVel*/
    public void updateInputs(ActuatorShoulderIOInputs inputs) {
            // shoulderDegrees += shoulder.getAngularVelocityRadPerSec() * 0.02;
            shoulderDegrees = Math.toDegrees(shoulder.getAngleRads());
            shoulderAngVel = Math.toDegrees(shoulder.getVelocityRadPerSec());
            inputs.shoulderDegrees = shoulderDegrees; 
            inputs.shoulderAngVel = shoulderAngVel; 
            inputs.shoulderCurrent = shoulder.getCurrentDrawAmps();
            inputs.shoulderVoltage = inputVoltage;
            shoulder.update(0.02);
    }
    @Override
    /**Sets shoulder voltage*/
    public void setVoltage(double voltage){
        shoulder.setInputVoltage(voltage);
        inputVoltage = voltage;
    }

    @Override
    /**Sets shoulder to a specific state based on angle (converts to radians) and velocity*/
    public void setAngle(double angle, double velocity){
        shoulder.setState((angle * Math.PI)/180, 0.0);
    }
}
