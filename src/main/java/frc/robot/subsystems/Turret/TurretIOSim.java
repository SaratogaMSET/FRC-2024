package frc.robot.subsystems.Turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO{
    //TODO: Change gearings
    SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getFalcon500(1), 150, 0.05, 0.5, 15.0/60, 55.0/60, false, 15.0/60);

    double voltage = 0;
    @Override
    public void updateInputs(TurretIOInputs inputs){
        inputs.phi = sim.getAngleRads();
        inputs.phiRadPerSec = sim.getVelocityRadPerSec();
        
        inputs.voltage = voltage;
        inputs.current = sim.getCurrentDrawAmps();
    }
    @Override
    public void setVoltage(double voltage){
        this.voltage = voltage;
        sim.setInputVoltage(voltage);
    }
    @Override
    public void setDesiredPhi(double radians, double radiansPerSecond){
        sim.setState(radians, radiansPerSecond);
    }
}
