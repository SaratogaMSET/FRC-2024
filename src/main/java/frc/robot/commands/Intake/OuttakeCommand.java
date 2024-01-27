package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ForearmSubsystem;

public class ExtakeDefaultCommand extends Command{
    ForearmSubsystem intake;
    double speed;
    public IntakeDefaultCommand(ForearmSubsystem intake, double speed){
        this.intake = intake;
        addRequirements(intake);
        this.speed = speed;
    }

    /*@Override
    public void initialize(){
        intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,30, 40, 2));
    }*/

    @Override
    public void execute(){
        intake.gravityCompensation();
        wheelIntake.set(-speed);
    }

    @Override
    public void end(boolean interrupted){
        intake.set(0);
        //intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,15, 40, 2));
    }
}