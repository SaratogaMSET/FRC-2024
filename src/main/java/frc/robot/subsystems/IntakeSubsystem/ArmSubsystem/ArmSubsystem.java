package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ShoulderSubsystem.ShoulderSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ShoulderSubsystem.ShoulderSubsystemIO.ShoulderSubsystemIOInputs;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.Wrist.WristSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.Wrist.WristSubsystemIO.WristSubsystemIOInputs;

public class ArmSubsystem extends SubsystemBase {
    ShoulderSubsystemIO shoulder;
    WristSubsystemIO wrist;
    double wristVelocity = 0;
    double wristAngle = 0;
    double shoulderVelocity = 0;
    double shoulderAngle = 0;
    double shoulderAngVel = 0.0;
    ArmState armState;
    ShoulderSubsystemIOInputs shoulderIOInputs = new ShoulderSubsystemIOInputs();
    WristSubsystemIOInputs wristIOInputs = new WristSubsystemIOInputs();
    

    public ArmSubsystem(ShoulderSubsystemIO shoulder, WristSubsystemIO wrist) {
        this.shoulder = shoulder;
        this.wrist = wrist;
    }

    /**
     * Using the current armState, run the arm TODO: Change speed magnitudes
     */
    public void runArm (){
         if (armState == null) {
            System.out.println("WARNING: ArmState not set!");
        } else {
            switch (armState) {
                case GROUND_DEPLOY:
                    shoulder.setAngle(Arm.SHOULDER_LOW_BOUND, 0.0);
                    wrist.setAngle(Arm.WRIST_LOW_BOUND, 0.01);
                    break;
                case AMP:
                    shoulder.setAngle(Arm.AmpScoringPositions.AMP_SHOULDER_ANGLE, 0.01);
                    wrist.setAngle(Arm.AmpScoringPositions.AMP_WRIST_ANGLE, 0.01);
                    break;
                case SOURCE:
                    shoulder.setAngle(Arm.SourceScoringPositions.SOURCE_SHOULDER_ANGLE, 0.01);
                    wrist.setAngle(Arm.SourceScoringPositions.SOURCE_WRIST_ANGLE, 0.01);
                    break;
                case NEUTRAL:
                    if (shoulderIOInputs.shoulderDegrees > Arm.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE) {
                        shoulder.setAngle(Arm.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        wrist.setAngle(Arm.GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    } else {
                        shoulder.setAngle(Arm.GroundNeutralPerimeterConstants.LOWER_MOTION_SHOULDER_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        wrist.setAngle(Arm.GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    }
                    break;
                case TRAP:
                    shoulder.setAngle(Arm.TrapScoringPositions.TRAP_SHOULDER_ANGLE, 0.01);
                    wrist.setAngle(Arm.TrapScoringPositions.TRAP_WRIST_ANGLE, 0.01);
                    break;
                case MANUAL:
                    break;
            }
        }
    }

    public void setVoltage(double shoulderVoltage, double wristVoltage){
        shoulder.setVoltage(shoulderVoltage);
        wrist.setVoltage(wristVoltage);
    }

    // Set the target arm state
    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    @Override
    public void simulationPeriodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        runArm();
    }

    @Override
    public void periodic() {
        shoulder.updateInputs(shoulderIOInputs);
        wrist.updateInputs(wristIOInputs);
        runArm();
    }
}