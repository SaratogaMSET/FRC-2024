package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ForearmSubsystem;

public class IntakeDefaultCommand extends Command{
    ForearmSubsystem intake;
    public IntakeDefaultCommand(ForearmSubsystem intake){
        this.intake = intake;
       
        addRequirements(intake);
    }


    @Override
    public void execute(){
        // if(actuator.getCurrent() > GroundIntake.currentLimit){
        //     actuator.setVoltageActuator(0);
        // }
        // else{
            intake.gravityCompensation();
        // }
    }
}