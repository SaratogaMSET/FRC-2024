package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure.Intake.Roller.RollerSubsystem;

public class RollerToShooterIRWithoutShooter extends Command {

  RollerSubsystem roller;
  double voltage;

  public RollerToShooterIRWithoutShooter(RollerSubsystem roller, double voltage) {
    this.roller = roller;
    this.voltage = voltage;

    addRequirements(roller);
  }

  @Override
  public void initialize() {
    roller.setIntakeFeederMode(true);
  }

  @Override
  public void execute() {
    if (roller.getCarriageBeamBreak()) {
      roller.setIntakeFeederVoltage(4);
    } else {
      roller.setIntakeFeederVoltage(voltage);
    }
    roller.setShooterFeederVoltage(2);
  }

  @Override
  public void end(boolean interrupted) {
    roller.setIntakeFeederVoltage(0);
    roller.setShooterFeederVoltage(0);
  }

  @Override
  public boolean isFinished() {
    return roller.getShooterBeamBreak(); // True, we stop rolling!
  }
}
