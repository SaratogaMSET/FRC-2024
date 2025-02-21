package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import java.util.function.DoubleSupplier;

public class RollerCommand extends Command {
  RollerSubsystem roller;
  double voltage;
  double speed = 0;
  boolean ampIntake = true;
  DoubleSupplier shoulderPosition;

  public RollerCommand(
      RollerSubsystem roller, double voltage, boolean ampIntake, DoubleSupplier shoulderPosition) {
    this.roller = roller;
    this.voltage = voltage;
    this.ampIntake = ampIntake;
    this.shoulderPosition = shoulderPosition;
    addRequirements(roller);
  }

  @Override
  public void initialize() {
    if (ampIntake) {
      roller.setIntakeFeederMode(true);
    } else {
      roller.setIntakeFeederMode(false);
    }
    roller.setShooterFeederMode(true);
    if (roller.getIntakeBeamBreak()) {
      roller.setShooterFeederVoltage(voltage);
    }
  }

  @Override
  public void execute() {
    roller.setIntakeFeederVoltage(voltage);
    boolean carriageTriggered = roller.getCarriageBeamBreak();
    if (!ampIntake) {
      if (voltage >= 0.0) {
        if (carriageTriggered) { // && previousIntakeTriggered == true){
          roller.setShooterFeederVoltage(1.3); // Change these values to also increase, was at: 2.4
          roller.setIntakeFeederVoltage(4); // was at: 4
        }
      }
    } else {
      roller.setShooterFeederVoltage(0.0);
    }
    // previousIntakeTriggered = intakeTriggered;
  }

  public void end(boolean interrupted) {
    roller.setIntakeFeederVoltage(0);
    roller.setShooterFeederVoltage(0);
  }

  @Override
  public boolean isFinished() {
    if (ampIntake) {
      return roller.getIntakeBeamBreak() && shoulderPosition.getAsDouble() < 0.0;
    } else {
      if (voltage > 0) {
        return roller.getShooterBeamBreak();
      } // else if (voltage == 0){
      //    return true;
      // }
      return false;
    }
  }
}
