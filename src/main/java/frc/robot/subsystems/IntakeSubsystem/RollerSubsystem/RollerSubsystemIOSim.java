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
  public void updateInputs(RollerSubsystemIOInputs inputs) {
    inputs.velocity = velocity;
    inputs.shooterIR = exited();
    inputs.rollerIR = acquired();
    update();
  }

  @Override
  public void roll(double speed) {
    velocity = speed;
  }

  @Override
  public double getSpeed() {
    return velocity;
  }

  public boolean acquired() {
    return enterIRGate;
  }

  public boolean exited() {
    return exitIRGate;
  }
}
