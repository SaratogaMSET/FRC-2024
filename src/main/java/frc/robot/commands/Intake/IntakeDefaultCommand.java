package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorSubsystem;

public class ActuatorDefaultCommand extends Command{
    ActuatorSubsystem actuator;
    public ActuatorDefaultCommand(ActuatorSubsystem actuatorSubsystem){
        actuator = actuatorSubsystem;
       
        addRequirements(actuatorSubsystem);
    }


    @Override
    public void execute(){
        // if(actuator.getCurrent() > GroundIntake.currentLimit){
        //     actuator.setVoltageActuator(0);
        // }
        // else{
            actuator.gravityCompensation();
        // }
    }
}