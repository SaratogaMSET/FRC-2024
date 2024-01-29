// package frc.robot.commands.Intake;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.IntakeSubsystem;
// import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;

// public class OuttakeCommand extends Command{
//     RollerSubsystem intake;
//     double speed;
//     public OuttakeCommand(RollerSubsystem intake, double speed){
//         this.intake = intake;
//         addRequirements(intake);
//         this.speed = speed;
//     }

//     /*@Override
//     public void initialize(){
//         intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,30, 40, 2));
//     }*/

//     @Override
//     public void execute(){
//         // intake.gravityCompensation();
//         intake.roll(-speed);
//     }

//     @Override
//     public void end(boolean interrupted){
//         intake.roll(0);
//         //intake.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,15, 40, 2));
//     }
// }
