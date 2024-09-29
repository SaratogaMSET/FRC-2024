package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterVisualizer {
  private final String logKey;
  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechanismRoot;
  private final MechanismLigament2d turretLigament;
  private final MechanismLigament2d pivotLigament;

  DoubleSupplier turretAngle;
  DoubleSupplier pivotAngle;

  public ShooterVisualizer(
      String logKey,
      Color8Bit colorOverride,
      DoubleSupplier turretAngle,
      DoubleSupplier pivotAngle) {
    // Initialize the visual model
    this.turretAngle = turretAngle;
    this.pivotAngle = pivotAngle;
    this.logKey = logKey;
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray)); // TODO: replace elevator height
    mechanismRoot = mechanism.getRoot("SuperStructure", 2, 0);
    turretLigament =
        mechanismRoot.append(
            new MechanismLigament2d(
                "Turret", 1.0, 90, 6, new Color8Bit(Color.kBlack))); // length 1 just for viz
    pivotLigament =
        turretLigament.append(
            new MechanismLigament2d(
                "Pivot", 0.5, 90, 6, new Color8Bit(Color.kGreen))); // length 0.5 just for viz
  }

  public void updateSim() {
    double turretAngle = this.turretAngle.getAsDouble();
    double pivotAngle = this.pivotAngle.getAsDouble();

    pivotLigament.setAngle(pivotAngle);
    turretLigament.setAngle(turretAngle);
    Logger.recordOutput("Mech2d" + logKey, mechanism);

    var turretPose =
        new Pose3d(
            0,
            0.0,
            Units.inchesToMeters(8.75), // height to superstructure
            new Rotation3d(0.0, 0.0, Math.toRadians(turretAngle)));

    var pivotPose =
        turretPose.transformBy(
            new Transform3d(
                new Translation3d(
                    0.0, 0.0, 0.0 // TODO: have height to shooter pivot here
                    ),
                new Rotation3d(0.0, Math.toRadians(pivotAngle), 0.0)));

    Logger.recordOutput("Mech3d" + logKey, turretPose, pivotPose);
  }
}
