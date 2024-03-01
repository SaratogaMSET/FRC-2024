package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.Logger;

public class RollerIOSim implements RollerIO {
  private boolean IRGate = false;
  private double voltage = 0.0;

  public RollerIOSim() {}

  public void update() {
    Logger.recordOutput("Mechanism2d/" + "Roller", voltage);
  }

  @Override
  /** Updates input for the roller speed, and shooter & roller IR */
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocity = voltage;
    inputs.rollerIR = false; //TODO: REWRITE THESE TO ACTUALLY WORK. PREVIOUS CODE DOES NOT ADHERE TO AKIT //acquired();
    update();
  }

  @Override
  public void setVoltage(double voltage) {
    this.voltage = voltage;
  }

}