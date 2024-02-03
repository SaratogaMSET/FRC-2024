 package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOTalon;
 import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO.ArmSubsystemIOInputs;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;

public class ManualWristCommand extends Command{
    ArmSubsystemIO wrist;
    ArmSubsystemIOInputs armIOInputs;
    double velocity = 0;
    double angle = 0;

    public ManualWristCommand(ArmSubsystemIO wrist, double speed, double angle){
        this.wrist = wrist;
        this.velocity = speed;
        this.angle = angle;
    }

    /*@Override
    public void initialize(){
        
    }*/

    @Override
    public void execute() {
        wrist.wristSetAngle(angle, velocity);
        wrist.updateInputs(armIOInputs);
    }

    /*@Override
    public void end(boolean interrupted){
        
    }*/

    @Override
    public boolean isFinished() {
        return true;
    }
}