package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants.Elevator;

public class ElevatorIOSim implements ElevatorIO {
  ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getFalcon500(2),
          Elevator.gearing,
          Elevator.carriageMassKg,
          Elevator.drumRadiusMeters,
          0.0,
          Elevator.HARD_LIMIT_HEIHT,
          true,
          0);
  double voltage = 0.0;

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.elevatorAppliedVolts = new double[] {voltage, 0.0};
    inputs.elevatorCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.carriagePositionMeters = new double[] {sim.getPositionMeters(), sim.getPositionMeters()};
    inputs.secondStagePositionMeters =
        sim.getPositionMeters() > Units.inchesToMeters(11.375)
            ? sim.getPositionMeters() - Units.inchesToMeters(11.375)
            : 0.0;
    inputs.elevatorVelocityMetersPerSec = new double[] {sim.getVelocityMetersPerSecond(), 0.0};
    inputs.hallEffectTriggered = sim.hasHitLowerLimit();
    inputs.heightLimitTriggered = sim.hasHitUpperLimit();
    sim.update(0.02);
  }

  @Override
  public void setDesiredHeight(double desiredHeight) {
    sim.setState(desiredHeight, 0.0);
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void resetLeftEncoder() {
    sim.setState(0.0, 0.0);
  }

  @Override
  public void resetRightEncoder() {
    sim.setState(0.0, 0.0);
  }
}
