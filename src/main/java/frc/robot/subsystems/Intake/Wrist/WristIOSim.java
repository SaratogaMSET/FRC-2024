package frc.robot.subsystems.Intake.Wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.Constants.Intake.Wrist;

public class WristIOSim implements WristIO{
    
    private double rads = 0.0 ;
    private double radPerSec = 0.0;
    private double inputVoltage = 0.0;
    private boolean hallEffect = false;
    SingleJointedArmSim wrist = new SingleJointedArmSim(DCMotor.getNeo550(1), Wrist.GEAR_RATIO, Wrist.MOI, Wrist.ARM_LENGTH,
        Math.toRadians(Wrist.LOW_BOUND), Math.toRadians(Wrist.HIGH_BOUND),
        true, Math.toRadians(DesiredStates.Neutral.WRIST_ANGLE));
            
    @Override
    /**Updates inputs for wrist voltage, current and angle in degrees and angleVel*/
    public void updateInputs(WristIOInputs inputs) {
            // wristDegrees += wrist.getAngularVelocityRadPerSec() * 0.02;
            // wristAngVel = wrist.getAngularVelocityRadPerSec();
            inputs.rads = wrist.getAngleRads();
            inputs.radsPerSec = wrist.getVelocityRadPerSec();
            inputs.current = wrist.getCurrentDrawAmps();
            inputs.voltage = inputVoltage;
            inputs.hallEffect = rads >= Wrist.HIGH_BOUND;  // TODO: Make this the angle relative to the shoulder
            wrist.update(0.02);

    }
    @Override
    public void setVoltage(double voltage){
        inputVoltage = voltage;
        wrist.setInputVoltage(voltage);
    }

    @Override
    /**Sets angle to a specific state based on angle (converts to radians) and velocity*/
    public void setAngle(double angle, double velocity){
        wrist.setState(angle, velocity);
    }
}