package frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Intake.AcutatorConstants;

public class ActuatorShoulderIOSim implements ActuatorShoulderIO{

    private double shoulderDegrees = 0.0;
    private double shoulderAngVel = 0.0;
    private double inputVoltage = 0.0;
    SingleJointedArmSim shoulder = new SingleJointedArmSim(DCMotor.getFalcon500(1), 5 * 3 * 1.5 * 15/8, 0.5,
    AcutatorConstants.SHOULDER_LENGTH + AcutatorConstants.WRIST_LENGTH, Math.toRadians(AcutatorConstants.SHOULDER_LOW_BOUND),
        Math.toRadians(AcutatorConstants.SHOULDER_HIGH_BOUND), true, Math.toRadians(AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE));

    
    @Override
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
    public void setVoltage(double voltage){
        shoulder.setInputVoltage(voltage);
        inputVoltage = voltage;
    }
    @Override
    public void setAngle(double angle, double velocity){
        shoulder.setState((angle * Math.PI)/180, 0.0);
    }
}
