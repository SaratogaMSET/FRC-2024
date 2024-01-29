 package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOTalon;
 import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;

public class ManualArmCommand extends Command{
    ArmSubsystemIO arm;
    double wristVelocity = 0;
    double wristAngle = 0;
    double shoulderVelocity = 0;
    double shoulderAngle = 0;
    // double angle = 0;

    public ManualArmCommand(ArmSubsystemIO arm, double wristSpeed, double shoulderSpeed, double wristAngle, double shoulderAngle){
        this.arm = arm;
        this.shoulderVelocity = shoulderSpeed;
        this.wristVelocity = wristSpeed;
        this.wristAngle = wristAngle;
        this.shoulderAngle = shoulderAngle;
    }

    /*@Override
    public void initialize(){
        
    }*/

    @Override
    public void execute() {
        arm.wristSetAngle(wristAngle, wristVelocity);
        arm.shoulderSetAngle(shoulderAngle, shoulderVelocity);
        arm.updateInputs();
        // arm.update(wristAngle, shoulderAngle);
    }

    /*@Override
    public void end(boolean interrupted){
        
    }*/

    @Override
    public boolean isFinished() {
        return true;
    }


}