// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Shooter.ShooterCalculation;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Stream;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  @Setter private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  @Setter private static Supplier<Rotation2d> armAngleSupplier = Rotation2d::new;
  @Setter private static Supplier<Rotation2d> turretAngleSupplier = Rotation2d::new;
  private static final List<Translation2d> autoNotes = new ArrayList<>();
  @Setter private static boolean hasNote = true;

  /** Show all staged notes for alliance */
  public static void showAutoNotes() {
    if (autoNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes", new Pose3d[] {});
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = autoNotes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new)); // 3d Poses for Visualization on 3d Field
  }

  public static void clearAutoNotes() {
    autoNotes.clear();
  }

  /** Add all notes to be shown at the beginning of auto */
  public static void resetAutoNotes() {
    clearAutoNotes();
    for (int i = FieldConstants.NotePositions.StagingLocations.kNotesStartingBlueWing.length - 1;
        i >= 0;
        i--) {

      autoNotes.add(
          AllianceFlipUtil.apply(
              FieldConstants.NotePositions.StagingLocations.kNotesStartingBlueWing[i]
                  .getTranslation()
                  .toTranslation2d()));
    }
    for (int i = FieldConstants.NotePositions.StagingLocations.kNotesStartingMidline.length - 1;
        i >= 0;
        i--) {
      autoNotes.add(
          AllianceFlipUtil.apply(
              FieldConstants.NotePositions.StagingLocations.kNotesStartingMidline[i]
                  .getTranslation()
                  .toTranslation2d()));
    }
  }

  /**
   * Take note from staged note
   *
   * @param note Number of note starting with 0 - 2 being spike notes going from amp to source side
   *     <br>
   *     and 3 - 7 being centerline notes going from amp to source side.
   */
  //   public static void takeAutoNote(int note) {
  //     autoNotes.set(note, null);
  //     hasNote = true;
  //   }

  /** Shows the currently held note if there is one */
  public static void showHeldNotes() {
    if (hasNote) {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {getIndexerPose3d()});
    } else {
      Logger.recordOutput("NoteVisualizer/HeldNotes", new Pose3d[] {});
    }
  }

  public static void ejectNote() {
    hasNote = false;
  }

  public static Command shoot(ShooterCalculation solver, double[] shotParams) {
    return new ScheduleCommand( // Branch off and exit immediately
        Commands.defer(
                () -> {
                  final Timer timer = new Timer();
                  timer.start();
                  return Commands.run(
                          () ->
                              Logger.recordOutput(
                                  "NoteVisualizer/ShotNotes",
                                  new Pose3d[] {
                                    new Pose3d(
                                        solver
                                            .simulateShot(
                                                shotParams[0], shotParams[1], timer.get())[0],
                                        solver
                                            .simulateShot(
                                                shotParams[0], shotParams[1], timer.get())[1],
                                        solver
                                            .simulateShot(
                                                shotParams[0], shotParams[1], timer.get())[2],
                                        // Y, X, Z
                                        new Rotation3d(0, 0, 0))
                                  }))
                      .until(() -> timer.hasElapsed(shotParams[2] * 2))
                      .finallyDo(
                          () -> Logger.recordOutput("NoteVisualizer/ShotNotes", new Pose3d[] {}));
                },
                Set.of())
            .ignoringDisable(true));
  }

  // Pose of indexer in 3d Space. Accounts for drivetrain rotation and everything.
  // TODO: turret indexer may be unneccesary.
  public static Pose3d getIndexerPose3d() {
    Transform3d indexerTransform =
        new Transform3d(
                0.0,
                0.0,
                Units.inchesToMeters(8.75),
                new Rotation3d(0.0, -armAngleSupplier.get().getRadians(), 0.0))
            .plus(new Transform3d(Units.inchesToMeters(12) * 0.35, 0.0, 0.0, new Rotation3d()));
    return new Pose3d(robotPoseSupplier.get())
        .transformBy(indexerTransform)
        .transformBy(
            new Transform3d(
                0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, turretAngleSupplier.get().getRadians())));
  }
}
