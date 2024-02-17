 package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;

public class ManualArmCommand extends Command{
    IntakeSubsystem intake;
    double wristVelocity = 0;
    double shoulderVelocity = 0;
    private double commandScheduleLoopMsec = 0.02;

    public ManualArmCommand(IntakeSubsystem arm, double wristSpeed, double shoulderSpeed){
        this.intake = arm;
        this.shoulderVelocity = shoulderSpeed;
        this.wristVelocity = wristSpeed;
    }

    @Override
    public void execute() {
        // To avoid having to create an additional velocity setting function
        intake.wrist.setAngle(wristVelocity * commandScheduleLoopMsec, wristVelocity);
        intake.shoulder.setAngle(shoulderVelocity * commandScheduleLoopMsec, shoulderVelocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}