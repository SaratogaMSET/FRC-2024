package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO.ArmSubsystemIOInputs;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;

public class ArmSubsystem extends SubsystemBase {
    ArmSubsystemIO arm;
    double wristVelocity = 0;
    double wristAngle = 0;
    double shoulderVelocity = 0;
    double shoulderAngle = 0;
    double shoulderAngVel = 0.0;
    ArmState armState;
    ArmSubsystemIOInputs armIOInputs = new ArmSubsystemIOInputs();

    public ArmSubsystem(ArmSubsystemIO arm) {
        this.arm = arm;
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
                    arm.shoulderSetAngle(Arm.SHOULDER_LOW_BOUND, 0.0);
                    arm.wristSetAngle(Arm.WRIST_LOW_BOUND, 0.01);
                    break;
                case AMP:
                    arm.shoulderSetAngle(Arm.AmpScoringPositions.AMP_SHOULDER_ANGLE, 0.01);
                    arm.wristSetAngle(Arm.AmpScoringPositions.AMP_WRIST_ANGLE, 0.01);
                    break;
                case SOURCE:
                    arm.shoulderSetAngle(Arm.SourceScoringPositions.SOURCE_WRIST_ANGLE, 0.01);
                    arm.wristSetAngle(Arm.SourceScoringPositions.SOURCE_SHOULDER_ANGLE, 0.01);
                    break;
                case NEUTRAL:
                    if (arm.shoulderGetDegrees() > Arm.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE) {
                        arm.shoulderSetAngle(Arm.GroundNeutralPerimeterConstants.UPPER_MOTION_SHOULDER_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        arm.wristSetAngle(Arm.GroundNeutralPerimeterConstants.UPPER_MOTION_WRIST_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    } else {
                        arm.shoulderSetAngle(Arm.GroundNeutralPerimeterConstants.LOWER_MOTION_SHOULDER_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.SHOULDER_POWER_PERCENT);
                        arm.wristSetAngle(Arm.GroundNeutralPerimeterConstants.LOWER_MOTION_WRIST_ANGLE,
                                Arm.GroundNeutralPerimeterConstants.WRIST_POWER_PERCENT);
                    }
                    break;
                case TRAP:
                    arm.shoulderSetAngle(Arm.TrapScoringPositions.TRAP_WRIST_ANGLE, 0.01);
                    arm.wristSetAngle(Arm.TrapScoringPositions.TRAP_SHOULDER_ANGLE, 0.01);
                    break;
                case MANUAL:
                    break;
            }
        }
    }

    // Set the target arm state
    public void setArmState(ArmState armState) {
        this.armState = armState;
    }

    @Override
    public void simulationPeriodic() {
        arm.updateInputs(armIOInputs);
        runArm();
    }

    @Override
    public void periodic() {
        arm.updateInputs(armIOInputs);
        runArm();
    }
}