package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;

public class TrapCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  RollerSubsystem rollerSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  double shoulderAngle;
  double wristAngle;

  double previousResetTime = Timer.getFPGATimestamp();

  public TrapCommand(
      IntakeSubsystem intakeSubsystem,
      RollerSubsystem roller,
      ElevatorSubsystem elevator,
      double shoulderAngle,
      double wristAngle) {
    this.intakeSubsystem = intakeSubsystem;
    this.rollerSubsystem = roller;
    this.elevatorSubsystem = elevator;
    this.shoulderAngle = shoulderAngle;
    this.wristAngle = wristAngle;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (elevatorSubsystem.getAverageExtension() > 0.90) {
      intakeSubsystem.setAngleShoulderMotionMagic(shoulderAngle);
      if (Math.abs(
              Math.toDegrees(intakeSubsystem.shoulderGetRads()) - Math.toDegrees(shoulderAngle))
          <= Intake.Shoulder.POSITION_ERROR_TOLERANCE) {
        intakeSubsystem.setAngleWrist(wristAngle);
        if (Math.abs(Math.toDegrees(intakeSubsystem.wristGetRads()) - Math.toDegrees(wristAngle))
            <= 10) {
          rollerSubsystem.setIntakeFeederVoltage(-3);
        }
        // intakeSubsystem.setAngleShoulder(shoulderAngle);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // intakeSubsystem.setVoltages(0.0, 0.0);
  }
}
