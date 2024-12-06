package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {

  /* Roller Logic Class */
  public RollerIO io;
  RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

  public RollerSubsystem(RollerIO rollerIO) {
    this.io = rollerIO;
  }

  @AutoLogOutput
  public boolean hasNote() {
    return getShooterBeamBreak() || getCarriageBeamBreak();
  }

  public boolean getIntakeBeamBreak() {
    return rollerIOInputs.intakeIR;
  }

  public boolean getShooterBeamBreak() {
    return rollerIOInputs.shooterIR;
  }

  public boolean getShooterFeederBeamBreak() {
    return rollerIOInputs.shooterFeederIR;
  }

  public boolean getCarriageBeamBreak() {
    return rollerIOInputs.carriageIR;
  }

  public void setIntakeFeederVoltage(double voltage) {
    io.setIntakeFeederVoltage(voltage);
  }

  public void setShooterFeederVoltage(double voltage) {
    io.setShooterFeederVoltage(voltage);
  }
  /** Sets the feeder mode of the intake (true -> brake, false -> coast) */
  public void setIntakeFeederMode(boolean brake) {
    if (brake) {
      io.setIntakeFeederMode(brake);
    } else {
      io.setIntakeFeederMode(brake);
    }
  }
  /** Sets the feeder mode of the shooter feede (true -> brake, false -> coast) */
  public void setShooterFeederMode(boolean brake) {
    if (brake) {
      io.setShooterFeederMode(brake);
    } else {
      io.setShooterFeederMode(brake);
    }
  }

  public void set(double voltage) {
    setIntakeFeederVoltage(voltage);
    setShooterFeederVoltage(voltage);
  }

  public double getShooterVoltage() {
    return rollerIOInputs.shooterVoltage;
  }

  public double getIntakeVoltage() {
    return rollerIOInputs.intakeVoltage;
  }

  @Override
  public void periodic() {
    io.updateInputs(rollerIOInputs);
    Logger.processInputs(getName(), rollerIOInputs);

    if (this.getCurrentCommand() != null)
      Logger.recordOutput("Commands/RollerCurrentCommand", this.getCurrentCommand().getName());
  }

  @Override
  public void simulationPeriodic() {
    io.updateInputs(rollerIOInputs);
    Logger.processInputs(getName(), rollerIOInputs);

    if (this.getCurrentCommand() != null)
      Logger.recordOutput("Commands/RollerCurrentCommand", this.getCurrentCommand().getName());
  }
}
