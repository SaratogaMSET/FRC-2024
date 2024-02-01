// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystemIOSim implements ArmSubsystemIO {
    private final String logKey;
    private final Mechanism2d mechanism;
    private final MechanismRoot2d mechanismRoot;
    private final MechanismLigament2d elevatorLigament;
    private final MechanismLigament2d shoulderLigament;
    private final MechanismLigament2d wristLigament;

    private double shoulderDegrees = 0.0;
    private double wristDegrees = 0.0;
    private double elevatorHeight = 0.0;
    private double shoulderAngVel = 0.0;
    private double wristAngVel = 0.0;
    private double elevatorVel = 0.0;
    private ArmState armState = ArmState.NEUTRAL;

    // TODO: Move these all to Constants
    double previousError = 0; // Move to constants, preferably in nested class within Arm class
    double errorDT;
    double prevError;

    public ArmSubsystemIOSim(String logKey, Color8Bit colorOverride) {
        this.logKey = logKey;
        mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
        mechanismRoot = mechanism.getRoot("Arm", 2 + 0, 0); //config.origin.getX()
        elevatorLigament =
            mechanismRoot.append(
                new MechanismLigament2d(
                    "Elevator", Arm.Sim.ELEVATOR_LENGTH, Arm.Sim.ELEVATOR_ANGLE, 6, new Color8Bit(Color.kBlack))); //TODO: Fix elevator length
        shoulderLigament =
            elevatorLigament.append(
                new MechanismLigament2d(
                    "Shoulder",
                    Arm.Sim.SHOULDER_LENGTH, //config.shoulder().length()
                    Arm.Sim.SHOULDER_ANGLE,
                    4,
                    colorOverride != null ? colorOverride : new Color8Bit(Color.kDarkBlue)));
        wristLigament =
            shoulderLigament.append(
                new MechanismLigament2d(
                    "Wrist",
                    Arm.Sim.SHOULDER_LENGTH, //config.wrist().length()
                    Arm.Sim.SHOULDER_ANGLE,
                    4,
                    colorOverride != null ? colorOverride : new Color8Bit(Color.kBlue)));
      }

    public void update(double shoulderAngle, double wristAngle) {
        shoulderLigament.setAngle(shoulderAngle - 90.0);
        wristLigament.setAngle(wristAngle);
        Logger.recordOutput("Mechanism2d/" + logKey, mechanism);
    }

    public void logRectangles(String logKey, double[][] rects, Color8Bit color) {
        var mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));

        for (int i = 0; i < rects.length; i++) {
            var rect = rects[i];
            var root = mechanism.getRoot("Rect" + Integer.toString(i), 2 + rect[0], rect[1]);
            var bottomLigament = root.append(new MechanismLigament2d("Bottom", rect[2] - rect[0], 0, 1, color));
            var rightLigament = bottomLigament
                    .append(new MechanismLigament2d("Right", rect[3] - rect[1], 90, 1, color));
            var topLigament = rightLigament.append(new MechanismLigament2d("Right", rect[2] - rect[0], 90, 1, color));
            topLigament.append(new MechanismLigament2d("Right", rect[3] - rect[1], 90, 1, color));
        }

        Logger.recordOutput("Mechanism2d/" + logKey, mechanism);
    }

    @Override
    public ArmSubsystemIOInputsAutoLogged updateInputs() {
        var inputs = new ArmSubsystemIOInputsAutoLogged();

        // Update physics "sim"
        shoulderDegrees += shoulderAngVel * 0.020;
        wristDegrees += wristAngVel * 0.020;
        elevatorHeight += elevatorVel * 0.020;

        inputs.shoulderDegrees = shoulderDegrees; // 0.020 because that's the RIO cycletime
        inputs.wristDegrees = wristDegrees;
        inputs.elevatorHeight = elevatorHeight; 

        update(shoulderDegrees, wristDegrees);

        return inputs;
    }

    // @Override
    // public void setDesiredState(SwerveModuleState state, boolean isOpenLoop) {
    // velocitySetpoint = state.speedMetersPerSecond;
    // steerSetpoint = state.angle.getRadians();
    // }

    /**
     * 
     * @return
     */
    @Override
    public double shoulderGetRadians() {
        return Math.toRadians(shoulderDegrees);
    }

    /**
     * 
     * @return
     */
    @Override
    public double shoulderGetDegrees() {
        return shoulderDegrees;
    }

    /**
     * 
     * @return
     */
    @Override
    public double wristGetRadians() {
        return Math.toRadians(wristDegrees);
    }

    /**
     * 
     * @return
     */
    @Override
    public double wristGetDegrees() {
        return wristDegrees;
    }

    /**
     * 
     * @return
     */
    @Override
    public double shoulderGetCurrent() {
        return 0.0; // FIXME: frick
    }

    /**
     * 
     * @return
     */
    @Override
    public double shoulderGetVoltage() {
        return 0.0; // FIXME: frick
    }

    /**
     * 
     * @param angle
     * @param powerPercent
     */
    @Override
    public void shoulderSetAngle(double angle, double powerPercent) {
        shoulderDegrees = angle;
    }

    /**
     * 
     * @param angle
     * @param powerPercent
     */
    @Override
    public void wristSetAngle(double angle, double powerPercent) {
        wristDegrees = angle;
    }

    /**
     * 
     */
    @Override
    public void gravityCompensation() {
        shoulderAngVel = Arm.PIDConstants.k_G * Math.cos(wristGetRadians() + Arm.WRIST_ENCODER_OFFSET_FROM_ZERO);
    }

    /**
     * 
     * @return
     */
    @Override
    public ArmState getArmState() {
        return armState;
    }

    /**
     * 
     * @param state
     */
    @Override
    public void setArmState(ArmState state) {
        armState = state;
    }

}