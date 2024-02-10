// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem;

import frc.robot.Constants;
import frc.robot.Constants.Intake;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

public class IntakeVisualizer {
    private final String logKey;
    private final Mechanism2d mechanism;
    private final MechanismRoot2d mechanismRoot;
    private final MechanismLigament2d elevatorLigament;
    private final MechanismLigament2d shoulderLigament;
    private final MechanismLigament2d wristLigament;

    private double shoulderDegrees = 0.0;
    private double wristDegrees = 0.0;

    double previousError = 0;
    double errorDT;
    double prevError;

    public IntakeVisualizer(String logKey, Color8Bit colorOverride) {
        // Initialize the visual model
        this.logKey = logKey;
        mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));  // TODO: replace elevator height
        mechanismRoot = mechanism.getRoot("Intake", 2 + 0, 0);
        elevatorLigament =
            mechanismRoot.append(
                new MechanismLigament2d(
                    "Elevator", Intake.AcutatorConstants.ELEVATOR_LENGTH, Intake.AcutatorConstants.ELEVATOR_ANGLE, 6, new Color8Bit(Color.kBlack))); //TODO: Fix elevator length
        shoulderLigament =
            elevatorLigament.append(
                new MechanismLigament2d(
                    "Shoulder",
                    Intake.AcutatorConstants.SHOULDER_LENGTH, //config.shoulder().length()
                    Intake.AcutatorConstants.SHOULDER_ANGLE,
                    4,
                    colorOverride != null ? colorOverride : new Color8Bit(Color.kDarkBlue)));
        wristLigament =
            shoulderLigament.append(
                new MechanismLigament2d(
                    "Wrist",
                    Intake.AcutatorConstants.SHOULDER_LENGTH, //config.wrist().length()
                    Intake.AcutatorConstants.SHOULDER_ANGLE,
                    4,
                    colorOverride != null ? colorOverride : new Color8Bit(Color.kBlue)));
      }

    public void updateSim(double shoulderAngle, double wristAngle) {
        shoulderLigament.setAngle(shoulderAngle - 90.0);
        shoulderDegrees = shoulderAngle - 90.0;
        wristLigament.setAngle(wristAngle);
        wristDegrees = wristAngle;
        Logger.recordOutput("Mech2d" + logKey, mechanism);
         var shoulderPose =
            new Pose3d(
                0,
                0.0,
                0.0,
                new Rotation3d(0.0, -shoulderAngle, 0.0));
        var wristPose =
            shoulderPose.transformBy(
                new Transform3d(
                    new Translation3d(Constants.Intake.AcutatorConstants.SHOULDER_LENGTH, 0.0, 0.0),
                    new Rotation3d(0.0, -wristAngle, 0.0)));
        Logger.recordOutput("Mech3d" + logKey, shoulderPose, wristPose);
    }
  public double shoulerAngle(){
    return shoulderDegrees;
  }
  public double wristAngle(){
    return wristDegrees;
  }
}