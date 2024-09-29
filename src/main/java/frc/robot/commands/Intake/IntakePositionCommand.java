package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakePositionCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  double shoulderAngle;
  double wristAngle;

  public IntakePositionCommand(
      IntakeSubsystem intakeSubsystem, double shoulderAngle, double wristAngle) {
    this.intakeSubsystem = intakeSubsystem;
    this.shoulderAngle = shoulderAngle;
    this.wristAngle = wristAngle;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {
    intakeSubsystem.wristIOInputs.previouslyZeroed = false;
  }

  @Override
  public void execute() {
    intakeSubsystem.setAngleShoulderMotionMagic(shoulderAngle);
    if (Math.abs(Math.toDegrees(intakeSubsystem.shoulderGetRads()) - Math.toDegrees(shoulderAngle))
        <= Intake.Shoulder.POSITION_ERROR_TOLERANCE) {
      intakeSubsystem.setAngleWrist(wristAngle);
      // intakeSubsystem.setAngleShoulder(shoulderAngle);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setVoltages(0.0, 0.0);
  }
}
