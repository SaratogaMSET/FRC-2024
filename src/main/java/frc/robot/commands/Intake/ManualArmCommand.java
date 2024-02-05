 package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystem;

public class ManualArmCommand extends Command{
    ArmSubsystem arm;
    double shoulderVoltage = 0;
    double wristVoltage = 0;
    private double commandScheduleLoopMsec = 0.02;

    public ManualArmCommand(ArmSubsystem arm, double shoulderVoltage, double wristVoltage){
        this.arm = arm;
        this.shoulderVoltage = shoulderVoltage;
        this.wristVoltage = wristVoltage;
    }

    @Override
    public void execute() {
        // To avoid having to create an additional velocity setting function
        arm.setVoltage(shoulderVoltage, wristVoltage);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}