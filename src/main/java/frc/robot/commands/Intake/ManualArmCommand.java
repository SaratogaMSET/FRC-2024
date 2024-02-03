 package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
 import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;

public class ManualArmCommand extends Command{
    ArmSubsystemIO arm;
    double wristVelocity = 0;
    double shoulderVelocity = 0;
    private double commandScheduleLoopMsec = 0.02;

    public ManualArmCommand(ArmSubsystemIO arm, double wristSpeed, double shoulderSpeed){
        this.arm = arm;
        this.shoulderVelocity = shoulderSpeed;
        this.wristVelocity = wristSpeed;
    }

    @Override
    public void execute() {
        // To avoid having to create an additional velocity setting function
        arm.wristSetAngle(wristVelocity * commandScheduleLoopMsec, wristVelocity);
        arm.shoulderSetAngle(shoulderVelocity * commandScheduleLoopMsec, shoulderVelocity);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}