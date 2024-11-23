package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class IntakeNeutralCommand extends Command {
  IntakeSubsystem intakeSubsystem;
  double shoulderAngle;
  double wristAngle;

  double previousResetTime = Timer.getFPGATimestamp();
  BooleanSupplier gunnerResetWrist;

  public IntakeNeutralCommand(IntakeSubsystem intakeSubsystem, BooleanSupplier gunnerResetWrist) {
    this.intakeSubsystem = intakeSubsystem;
    this.gunnerResetWrist = gunnerResetWrist;
    addRequirements(this.intakeSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    /* Does not run in Test. Reset Logic = if more than 3 seconds before reset or gunner presses the reset button */
    if (!DriverStation.isTest()) {
      intakeSubsystem.setAngleShoulder(Neutral.SHOULDER_ANGLE);

      if (Timer.getFPGATimestamp() - previousResetTime > 3 || gunnerResetWrist.getAsBoolean()) {
        Logger.recordOutput("WristDefaultTimer", "Reset");
        intakeSubsystem.setPreviousZeroed(false);
      }

      /* Not zeroed but we are within tolerances...*/
      if (intakeSubsystem.getCurrentLimitTripped()
          /*
          Manual fucking current Limit lmao. If Output current >
              Magic number max current tolerance =>
              we are pushing against the hardstop
          */
          && intakeSubsystem.getPreviousZeroed() == false
      // && intakeSubsystem.getWristEncoderPosition()
      // && intakeSubsystem.wrist.motor.getEncoder().getPosition()
      // < 0.07
      ) { // Inside Tolerance(No need to zero)
        /* Reset Encoders */
        previousResetTime = Timer.getFPGATimestamp();
        intakeSubsystem.setPreviousZeroed(true);
        intakeSubsystem.setWristVoltage(0.0);
        intakeSubsystem.setWristEncoderPosition(0);
        Logger.recordOutput(
            "WristDefaultState", "Output Current > Current Tolerance = Pushing against hardstop");
      } else if (intakeSubsystem
          .getPreviousZeroed()) { // The wrist is zeroed... move to the zero position.
        intakeSubsystem.setAngleWrist(0);
        Logger.recordOutput("WristDefaultState", "Wrist Is Previously Zeroed");
      } else {
        Logger.recordOutput("WristDefaultState", "Wrist is Moving Downward");
        intakeSubsystem.setWristVoltage(-2); // -1.5
      }

    } else {
      intakeSubsystem.setVoltages(0, 0);
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
