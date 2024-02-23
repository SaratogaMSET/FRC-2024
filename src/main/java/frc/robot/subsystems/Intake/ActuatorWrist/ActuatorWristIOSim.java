package frc.robot.subsystems.Intake.ActuatorWrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.Constants.Intake.Wrist;

public class ActuatorWristIOSim implements ActuatorWristIO{
    
    private double wristDegrees = 0.0 ;
    private double wristAngVel = 0.0;
    private double inputVoltage = 0.0;
    SingleJointedArmSim wrist = new SingleJointedArmSim(DCMotor.getNeo550(1), Wrist.GEAR_RATIO, Wrist.MOI, Wrist.ARM_LENGTH,
        Math.toRadians(Wrist.LOW_BOUND), Math.toRadians(Wrist.HIGH_BOUND),
        true, Math.toRadians(DesiredStates.Neutral.WRIST_ANGLE));
            
    @Override
    /**Updates inputs for wrist voltage, current and angle in degrees and angleVel*/
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
    /**Sets wrist voltage*/
    public void setVoltage(double voltage){
        inputVoltage = voltage;
        wrist.setInputVoltage(voltage);
    }

    @Override
    /**Sets angle to a specific state based on angle (converts to radians) and velocity*/
    public void setAngle(double angle, double velocity){    //TODO: Remove velocity it doesn't do anything
        wrist.setState((angle * Math.PI)/180, 0.0);
    }
}
