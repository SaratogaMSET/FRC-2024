 package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class IntakeManualCommand extends Command{
    IntakeSubsystem intake;
    double wristVelocity = 0;
    double shoulderVelocity = 0;
    private double commandScheduleLoopSec = 0.02;

    public IntakeManualCommand(IntakeSubsystem arm, double wristSpeed, double shoulderSpeed){
        this.intake = arm;
        this.shoulderVelocity = shoulderSpeed;
        this.wristVelocity = wristSpeed;
    }

    @Override
    public void execute() {
        intake.wrist.setAngle(intake.wristGetRads() + wristVelocity * commandScheduleLoopSec, wristVelocity);
        intake.shoulder.setAngle(intake.shoulderGetRads() + shoulderVelocity * commandScheduleLoopSec, shoulderVelocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}