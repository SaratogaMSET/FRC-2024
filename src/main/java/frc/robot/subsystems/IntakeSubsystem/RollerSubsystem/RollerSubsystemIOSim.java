package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import org.littletonrobotics.junction.Logger;

public class RollerSubsystemIOSim implements RollerSubsystemIO {
  private boolean enterIRGate = false;
  private boolean exitIRGate = false;
  private double velocity = 0.0;

  public RollerSubsystemIOSim() {}

  public void update() {
    Logger.recordOutput("Mechanism2d/" + "Roller", velocity);
  }

  @Override
  /** Updates input for the roller speed, and shooter & roller IR */
  public void updateInputs(RollerSubsystemIOInputs inputs) {
    inputs.velocity = velocity;
    inputs.shooterIR = false;//TODO: REWRITE THESE TO ACTUALLY WORK. PREVIOUS CODE DOES NOT ADHERE TO AKIT // exited();
    inputs.rollerIR = false; //TODO: REWRITE THESE TO ACTUALLY WORK. PREVIOUS CODE DOES NOT ADHERE TO AKIT //acquired();
    update();
  }

  @Override
  public void roll(double speed) {
    velocity = speed;
  }

}