// package frc.robot.commands.Intake;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
// import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOTalon;
// import frc.robot.Constants.IntakeSubsystem.Forearm;
// import frc.robot.Constants.IntakeSubsystem.Roller;
// import frc.robot.Constants.IntakeSubsystem.Forearm.ArmState;

// public class GroundDeployCommand extends Command {
//     RollerSubsystem roller;
//     ArmSubsystemIOTalon forearm;
//     // boolean useIRGate = true;

//     public GroundDeployCommand(RollerSubsystem roller, ArmSubsystemIOTalon forearm) {
//         this.roller = roller;
//         this.forearm = forearm;
//         addRequirements(forearm, roller);
//     }

//     @Override
//     public void execute() {
//         // If the forearm is still above the wrist movement threshold (to stay in perimeter.
// otherwise run as normal)
//         //roller.roll(Roller.ROLLING_SPEED);
//         forearm.setArmState(ArmState.NEUTRAL);
//         /*if (forearm.getArmState() == ArmState.NEUTRAL
//                 && forearm.elbowGetDegrees() >
// Forearm.GroundNeutralPerimeterConstants.UPPER_MOTION_ELBOW_ANGLE) {
//
// forearm.elbowSetAngle(Forearm.GroundNeutralPerimeterConstants.UPPER_MOTION_ELBOW_ANGLE,
//                     Forearm.GroundNeutralPerimeterConstants.ELBOW_POWER_PERCENT);
//
// forearm.wristSetAngle(Forearm.GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE,
//                     Forearm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
//         } else {
//
// forearm.elbowSetAngle(Forearm.GroundNeutralPerimeterConstants.LOWER_MOTION_ELBOW_ANGLE,
//                     Forearm.GroundNeutralPerimeterConstants.ELBOW_POWER_PERCENT);
//
// forearm.wristSetAngle(Forearm.GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE,
//                     Forearm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
//         }*/
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (roller.acquired()) {
//             roller.roll(Roller.HOLD_SPEED);
//         } else {
//             roller.roll(Roller.NEUTRAL_SPEED);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return roller.acquired();
//     }
// }
