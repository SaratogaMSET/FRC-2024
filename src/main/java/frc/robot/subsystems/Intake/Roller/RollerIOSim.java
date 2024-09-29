package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.Logger;

public class RollerIOSim implements RollerIO {
  private boolean IRGate = false;
  private double intakeVoltage = 0.0;
  private double shooterVoltage = 0.0;

  public RollerIOSim() {}

  public void update() {
    Logger.recordOutput("Mechanism2d/" + "Intake Roller", intakeVoltage);
    Logger.recordOutput("Mechanism2d/" + "Shooter Roller", shooterVoltage);
  }

  @Override
  /** Updates input for the roller speed, and shooter & roller IR */
  public void updateInputs(RollerIOInputs inputs) {
    inputs.intakeVoltage = intakeVoltage;
    inputs.shooterVoltage = shooterVoltage;
    inputs.intakeIR =
        false; // TODO: REWRITE THESE TO ACTUALLY WORK. PREVIOUS CODE DOES NOT ADHERE TO AKIT
    // //acquired();
    inputs.shooterIR = false;
    update();
  }

  @Override
  public void setIntakeFeederVoltage(double voltage) {
    this.intakeVoltage = voltage;
  }

  @Override
  public void setShooterFeederVoltage(double voltage) {
    this.shooterVoltage = voltage;
  }
}
