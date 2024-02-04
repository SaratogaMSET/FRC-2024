// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
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

    double previousError = 0;
    double errorDT;
    double prevError;

    public ArmSubsystemIOSim(String logKey, Color8Bit colorOverride) {
        // Initialize the visual model
        this.logKey = logKey;
        mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));  // TODO: replace elevator height
        mechanismRoot = mechanism.getRoot("Arm", 2 + 0, 0);
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

    public void updateSim(double shoulderAngle, double wristAngle) {
        shoulderLigament.setAngle(shoulderAngle - 90.0);
        wristLigament.setAngle(wristAngle);
        Logger.recordOutput("Mechanism2d/" + logKey, mechanism);
    }

    @Override
    public void updateInputs(ArmSubsystemIOInputs inputs) {
        // Update physics "sim"
        shoulderDegrees += shoulderAngVel * 0.020;
        wristDegrees += wristAngVel * 0.020;
        elevatorHeight += elevatorVel * 0.020;
        inputs.shoulderDegrees = shoulderDegrees; // 0.020 because that's the RIO cycletime
        inputs.wristDegrees = wristDegrees;
        inputs.elevatorHeight = elevatorHeight; 
        inputs.armState = armState;
        updateSim(shoulderDegrees, wristDegrees);
    }

    @Override
    public double shoulderGetRadians() {
        return Math.toRadians(shoulderDegrees);
    }

    @Override
    public double shoulderGetDegrees() {
        return shoulderDegrees;
    }

    @Override
    public double wristGetRadians() {
        return Math.toRadians(wristDegrees);
    }

    @Override
    public double wristGetDegrees() {
        return wristDegrees;
    }

    @Override
    public double shoulderGetCurrent() {
        return 0.0;
    }

    @Override
    public double shoulderGetVoltage() {
        return shoulderAngVel * 12.0;
    }

    @Override
    public void shoulderSetAngle(double angle, double powerPercent) {
        shoulderDegrees = angle;
    }

    @Override
    public void wristSetAngle(double angle, double powerPercent) {
        wristDegrees = angle;
    }

    @Override
    public void gravityCompensation() {}
}