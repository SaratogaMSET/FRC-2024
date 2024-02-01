package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOTalon;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Roller;
import frc.robot.Constants.IntakeSubsystem.Arm.AmpScoringPositions;
import frc.robot.Constants.IntakeSubsystem.Arm.TrapScoringPositions;
import frc.robot.Constants.IntakeSubsystem.Arm.SourceScoringPositions;
import frc.robot.Constants.IntakeSubsystem.Arm.GroundNeutralPerimeterConstants;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;

public class ArmSubsystem implements ArmSubsystemIO {
    ArmSubsystemIO arm;
    RollerSubsystemIO roller;
    double wristVelocity = 0;
    double wristAngle = 0;
    double shoulderVelocity = 0;
    double shoulderAngle = 0;

    public String armState (ArmState setArmState) {
        switch(setArmState){
            case GROUND_DEPLOY:
                arm.shoulderSetAngle(arm.SHOULDER_LOW_BOUND,100);
                arm.wristSetAngle(arm.WRIST_LOW_BOUND,100);
                roller.roll(Roller.ROLLING_SPEED);
                break;
            case AMP:
                arm.shoulderSetAngle(arm.AmpScoringPositions.AMP_SHOULDER_ANGLE,100);
                arm.wristSetAngle(arm.AmpScoringPositions.AMP_WRIST_ANGLE,100);
                roller.roll(Roller.ROLLING_SPEED);
                break;
            case SOURCE:
                arm.shoulderSetAngle(Arm.SourceScoringPositions.SOURCE_WRIST_ANGLE,100);
                arm.wristSetAngle(Arm.SourceScoringPositions.SOURCE_SHOULDER_ANGLE,100);
                roller.roll(Roller.ROLLING_SPEED);
                break;
            case NEUTRAL:
                roller.roll(Roller.ROLLING_SPEED);
                if(arm.shoulderGetDegrees() > arm.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE){
                    arm.shoulderSetAngle(arm.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE, arm.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                    arm.wristSetAngle(arm.GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE, arm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                }
                else {
                    arm.shoulderSetAngle(arm.GroundNeutralPerimeterConstants.LOWER_MOTION_SHOULDER_ANGLE, arm.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                    arm.wristSetAngle(arm.GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE, arm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                }
                break;
            case TRAP:
                arm.shoulderSetAngle(Arm.TrapScoringPositions.TRAP_WRIST_ANGLE,100);
                arm.wristSetAngle(Arm.TrapScoringPositions.TRAP_SHOULDER_ANGLE,100);
                roller.roll(Roller.ROLLING_SPEED);
                break;
        }

  //      String stateString = setArmState.toString();
  //      return stateString;
    }

  //  public void periodic() {
  //      SmartDashboard.putString("Arm State: ", armState());
  //  }





}