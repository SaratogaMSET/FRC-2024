package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Intake.AcutatorConstants;

public class ActuatorWristIOSim implements ActuatorWristIO{
    
    private double wristDegrees = 0.0 ;
    private double wristAngVel = 0.0;
    private double inputVoltage = 0.0;
    SingleJointedArmSim wrist = new SingleJointedArmSim(DCMotor.getNeo550(1), 50, 0.3, AcutatorConstants.WRIST_LENGTH,
        Math.toRadians(AcutatorConstants.WRIST_LOW_BOUND), Math.toRadians(AcutatorConstants.WRIST_HIGH_BOUND),
        true, Math.toRadians(AcutatorConstants.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE));
            
    @Override
    public void updateInputs(ActuatorWristIOInputs inputs) {
            // wristDegrees += wrist.getAngularVelocityRadPerSec() * 0.02;
            // wristAngVel = wrist.getAngularVelocityRadPerSec();
            wristDegrees = Math.toDegrees(wrist.getAngleRads());
            wristAngVel = Math.toDegrees(wrist.getVelocityRadPerSec());
            inputs.wristDegrees = wristDegrees;
            inputs.wristAngVel = wristAngVel;
            inputs.wristCurrent = wrist.getCurrentDrawAmps();
            inputs.wristVoltage = inputVoltage;
            wrist.update(0.02);

    }
    @Override
    public void setVoltage(double voltage){
        inputVoltage = voltage;
        wrist.setInputVoltage(voltage);
    }
    @Override
    public void setAngle(double angle, double velocity){
        wrist.setState((angle * Math.PI)/180, 0.0);
    }
}
