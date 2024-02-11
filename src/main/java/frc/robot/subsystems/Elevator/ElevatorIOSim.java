package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Sim;

public class ElevatorIOSim implements ElevatorIO {
    ElevatorSim sim = new ElevatorSim(DCMotor.getFalcon500(2), ElevatorConstants.gearing, ElevatorConstants.carriageMassKg,
    ElevatorConstants.drumRadiusMeters, 0.0 ,ElevatorConstants.SOFT_LIMIT_HEIGHT, true, 0);

    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.elevatorAppliedVolts = 0.0;
        inputs.elevatorCurrentAmps = sim.getCurrentDrawAmps();
        inputs.elevatorPositionMeters = sim.getPositionMeters();
        inputs.elevatorVelocityMetersPerSec = sim.getVelocityMetersPerSecond();
        inputs.hallEffectTriggered = sim.hasHitLowerLimit();
        inputs.heightLimitTriggered = sim.hasHitUpperLimit();
        sim.update(0.02);
    }

    @Override
    public void setDesiredHeight(double desiredHeight){
        sim.setState(desiredHeight, 0.0);
    }
    @Override
    public void setVoltage(double voltage){
        sim.setInputVoltage(voltage);
    }

}
