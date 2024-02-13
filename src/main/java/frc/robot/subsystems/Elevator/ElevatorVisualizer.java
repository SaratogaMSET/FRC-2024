
package frc.robot.subsystems.Elevator;

import frc.robot.Constants.ElevatorConstants;
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
import org.littletonrobotics.junction.Logger;

public class ElevatorVisualizer {
    private final String logKey;
    private final Mechanism2d mechanism;
    private final MechanismRoot2d mechanismRoot;
    private final MechanismLigament2d elevatorLigament;
    ElevatorSubsystem elevator;

    public ElevatorVisualizer(String logKey, Color8Bit colorOverride) {
        this.elevator = elevator;
        // Initialize the visual model
        this.logKey = logKey;
        mechanism = new Mechanism2d(4, 1.02235, new Color8Bit(Color.kGray));  //
        mechanismRoot = mechanism.getRoot("Intake", 2 + 0, 0); //TODO: OFFSET TUNING
        elevatorLigament =
            mechanismRoot.append(
                new MechanismLigament2d(
                    "Elevator", 0.0, 90, 4, new Color8Bit(Color.kBlack)));
    }

    public void updateSim(double elevatorLength) {
        elevatorLigament.setLength(elevatorLength);
        Logger.recordOutput("Mech2d" + logKey, mechanism);
         var elevatorPose =
            new Pose3d(
                0,
                0.0,
                elevatorLength,
                new Rotation3d(0.0, 0.0, 0.0));
        Logger.recordOutput("Mech3d" + logKey, elevatorPose);
    }

}