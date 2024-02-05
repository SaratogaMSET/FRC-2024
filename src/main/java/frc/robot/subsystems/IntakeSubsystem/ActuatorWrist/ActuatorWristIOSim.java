package frc.robot.subsystems.IntakeSubsystem.ActuatorWrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Intake.AcutatorConstants;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIO.ActuatorShoulderIOInputs;

public class ActuatorWristIOSim implements ActuatorWristIO{
    
    private double wristDegrees = 0.0;
    private double wristAngVel = 0.0;
    SingleJointedArmSim wrist = new SingleJointedArmSim(DCMotor.getNeo550(1), 50, 0.025, AcutatorConstants.WRIST_LENGTH,
        AcutatorConstants.WRIST_LOW_BOUND * Math.PI/180, AcutatorConstants.WRIST_HIGH_BOUND * Math.PI/180,
        true,0.0);
    
    @Override
    public void updateInputs(ActuatorWristIOInputs inputs) {
            wrist.update(0.02);
            // wristDegrees += wrist.getAngularVelocityRadPerSec() * 0.02;
            // wristAngVel = wrist.getAngularVelocityRadPerSec();
            wristDegrees = wrist.getAngleRads() * 180/Math.PI;
            wristAngVel = wrist.getVelocityRadPerSec() * 180/Math.PI;
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
    public void setAngle(double angle, double velocity){
        wrist.setState((angle * Math.PI)/180, 0.0);
    }
}
