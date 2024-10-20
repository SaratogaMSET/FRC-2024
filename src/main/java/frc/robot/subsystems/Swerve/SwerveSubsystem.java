// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.Swerve.Module.WHEEL_RADIUS;

import com.google.common.collect.Streams;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIO;
import frc.robot.subsystems.Vision.VisionIOReal;
import frc.robot.subsystems.Vision.VisionIOSim;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

public class SwerveSubsystem extends SubsystemBase {
  public static double MAX_LINEAR_SPEED = Units.feetToMeters(16.5); // 17.1 wihtout foc
  public static double TRACK_WIDTH_X = Units.inchesToMeters(18.5);
  public static double TRACK_WIDTH_Y = Units.inchesToMeters(18.5);
  public static double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final SysIdRoutine m_slipSysIdRoutine;
  private final PIDController driftCorrectionPID = new PIDController(0.1, 0.00, 0.000);
  private SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
  public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  private final Vision[] cameras;
  private boolean visionInAutonomous = false;
  private Pose2d targetPose = new Pose2d();
  private List<Pose2d> activePath = new ArrayList<Pose2d>();
  // private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();
  private Rotation2d gyroRotation = new Rotation2d();

  private boolean seeded = false;

  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private double pXY = 0;
  private double desiredHeading;
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d(),
          frc.robot.Constants.Vision.stateSTD,
          Constants.Vision.visDataSTD);

  public SwerveSubsystem(Vision[] visionIOs, GyroIO gyroIO, ModuleIO[] moduleIOs) {
    switch (Constants.getRobot()) {
      case ROBOT_2024C:
      case ROBOT_SIMBOT:
        MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
        TRACK_WIDTH_X = Units.inchesToMeters(18.5);
        TRACK_WIDTH_Y = Units.inchesToMeters(18.5);
        break;
      case ROBOT_2024P:
        MAX_LINEAR_SPEED = Units.feetToMeters(16.5);
        TRACK_WIDTH_X = Units.inchesToMeters(24.75);
        TRACK_WIDTH_Y = Units.inchesToMeters(24.75);
        break;
      default:
        MAX_LINEAR_SPEED = Units.feetToMeters(17.1);
        TRACK_WIDTH_X = Units.inchesToMeters(18.5);
        TRACK_WIDTH_Y = Units.inchesToMeters(18.5);
    }
    DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    cameras = new Vision[visionIOs.length];

    for (int i = 0; i < visionIOs.length; i++) {
      cameras[i] = visionIOs[i];
    }

    this.gyroIO = gyroIO;
    modules[0] = new Module(moduleIOs[0], 0);
    modules[1] = new Module(moduleIOs[1], 1);
    modules[2] = new Module(moduleIOs[2], 2);
    modules[3] = new Module(moduleIOs[3], 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1, 0.0, 0.0),
            new PIDConstants(0.5, 0.0, 0.0),
            MAX_LINEAR_SPEED,
            DRIVE_BASE_RADIUS,
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
    m_slipSysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.25).per(Seconds.of(1)),
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  for (int i = 0; i < 4; i++) {
                    modules[i].runCharacterization(voltage.in(Volts));
                  }
                },
                null,
                this));
  }

  public static Vision[] createCamerasReal() {
    return new Vision[] {
      new Vision(new VisionIOReal(0), 0)
    }; // , new Vision(new VisionIOReal(1), 1)};
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  public static Vision[] createCamerasSim() {
    return new Vision[] {new Vision(new VisionIOSim(0), 0), new Vision(new VisionIOSim(1), 1)};
  }

  public static Vision[] createVisionIOs() {
    return new Vision[] {new Vision(new VisionIO() {}, 0)};
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {

        SmartDashboard.putNumber(
            "CanCoder" + moduleIndex + "angle", modules[moduleIndex].getAngle().getDegrees());
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      Logger.recordOutput("Swerve-Sub Raw Gyro Rotation", rawGyroRotation);
      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    for (Vision camera : cameras) {
      camera.updateInputs(
          new Pose3d(
              poseEstimator
                  .getEstimatedPosition())); // Update all inputs with current pose estimator
      // position.
    }

    double prevTimestamp = 0;

    for (Vision camera : cameras) {
      Optional<EstimatedRobotPose> visionData = camera.inputs.estPose;
      double timestamp = camera.inputs.timestamp;

      if (!visionData.isPresent()) continue;

      Logger.recordOutput(
          "Raw Pose of Camera" + String.valueOf(camera.getIndex()),
          camera.inputs.pipelineResult.getMultiTagResult().estimatedPose.best);
      Logger.recordOutput(
          "Robot Center Pose of Camera" + String.valueOf(camera.getIndex()),
          visionData.get().estimatedPose); // Serialize for 3dField

      // If too far off the ground, or too far off rotated, we consider it as bad data.
      if (visionData.isPresent() && (Math.abs(visionData.get().estimatedPose.getZ()) > 0.25)
          // || Math.abs(visionData.get().estimatedPose.getRotation().getZ() -
          // getPose().getRotation().getRadians()) > 0.2 // TODO: Add this back? NGL THIS REALL
          // SHOULD WORK
          // || Math.abs(visionData.get().estimatedPose.getRotation().getY() - 0) >
          // Units.degreesToRadians(10)
          || Math.abs(visionData.get().estimatedPose.getRotation().getX() - 0)
              > Units.degreesToRadians(12)
          || visionData.get().targetsUsed.size() <= 1 // Only consider multitag. Thanks 8033.
          || (DriverStation.isAutonomous())) {
        Logger.recordOutput(
            "Vision Pitch(Degrees)",
            Units.radiansToDegrees(visionData.get().estimatedPose.getRotation().getY()));
        Logger.recordOutput(
            "Vision Roll(Degrees)",
            Units.radiansToDegrees(visionData.get().estimatedPose.getRotation().getX()));
        visionData = Optional.empty();
      }

      // if (camera.inputs.targetCount == 1 && camera.inputs.averageAmbiguity > 0.3) visionData =
      // Optional.empty();

      // visionData = Optional.empty();

      if (!visionData.isPresent()) continue;

      Pose2d inst_pose = visionData.get().estimatedPose.toPose2d();

      if (seeded == false) {
        seeded = true;
        poseEstimator.resetPosition(rawGyroRotation, modulePositions, inst_pose);
        Logger.recordOutput("Seed Pose", inst_pose);
      } else if (visionData.isPresent()
          // && getPose().getTranslation().getDistance(inst_pose.getTranslation()) < 5.06 *
          // (timestamp - prevTimestamp) * 1.25 // Fudged max speed(m/s) * timestamp difference *
          // 1.25. Probably doesn't work.
          // && timestamp > prevTimestamp // Please never use this
          && getPose().getTranslation().getDistance(inst_pose.getTranslation())
              > Units.inchesToMeters(2)
      // && (camera.inputs.pipelineResult.getBestTarget().getFiducialId() == 7 ||
      //       camera.inputs.pipelineResult.getBestTarget().getFiducialId() == 8)
      ) {
        // poseEstimator.addVisionMeasurement(inst_pose, timestamp);
        poseEstimator.addVisionMeasurement(
            inst_pose, timestamp, findVisionMeasurementStdDevs(visionData.get()));
        Logger.recordOutput("Vision Poses", inst_pose);
      }
      Logger.recordOutput("Vision Previous Timestamp", prevTimestamp);
      Logger.recordOutput("Vision Timestamp", timestamp);
      prevTimestamp = Math.max(timestamp, prevTimestamp);
    }
    Logger.recordOutput("Measured Field Relative Speeds", getFieldRelativeSpeeds());
    Logger.recordOutput("Raw Gyro Rotation", rawGyroRotation);
    // Logger.processInputs("Vision", );
  }

  public static Matrix<N3, N1> findVisionMeasurementStdDevs(EstimatedRobotPose estimation) {
    double sumDistance = 0;
    for (var target : estimation.targetsUsed) {
      var t3d = target.getBestCameraToTarget();

      sumDistance +=
          Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
    }

    double avgDistance = sumDistance / estimation.targetsUsed.size(); // R^2?

    var deviation =
        Constants.Vision.visDataSTD
            .times(avgDistance * Constants.Vision.distanceFactor)
            .plus(VecBuilder.fill(0, 0, 100)); // At two meters it starts trusting vision less!
    // Fudge rotation to not be considered.
    // TAG_COUNT_DEVIATION_PARAMS
    //     .get(
    //         MathUtil.clamp(
    //             estimation.targetsUsed.size() - 1, 0, TAG_COUNT_DEVIATION_PARAMS.size() - 1))
    //     .computeDeviation(avgDistance);

    return deviation;
  }

  private Optional<EstimatedRobotPose> calculateInHouseMultiTag(Vision camera) {
    PhotonPipelineResult result = camera.inputs.pipelineResult;
    if (result.getTargets().size() >= 1) {
      return Optional.empty();
    }

    Transform3d transform = result.getMultiTagResult().estimatedPose.best;
    Transform3d robotToCamera;
    if (camera.getIndex() == 0) robotToCamera = Constants.Vision.jawsCamera0;
    else robotToCamera = Constants.Vision.jawsCamera1;
    Pose3d best =
        new Pose3d()
            .plus(transform)
            .relativeTo(FieldConstants.aprilTags.getOrigin())
            .plus(robotToCamera.inverse());
    return Optional.of(
        new EstimatedRobotPose(
            best,
            result.getTimestampSeconds(),
            result.getTargets(),
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR));
  }

  // public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
  //   return this.runVelocityCmd(
  //       () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()));
  // }
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(getModuleStates()), rawGyroRotation);
  }

  public ChassisSpeeds emptyChassisSpeeds() {
    return new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  public Command runVelocityCmd(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> runVelocity(speeds.get()));
  }

  public Command runVelocityTeleopFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.run(
        () -> {
          //  Rotation2d rot = new Rotation2d(MathUtil.angleModulus(getRawGyroYaw()));
          var allianceSpeeds =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds.get(),
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                      ? getRotation() // maybe change to rot. The reason to use rawGyroRotation over
                      // rot is in case of gyro disconnect
                      : getRotation()
                          .plus(
                              new Rotation2d(
                                  Math.PI))); // rawGyroRotation.plus(new Rotation2d(Math.PI)));
          // //new
          // Rotation2d(MathUtil.angleModulus(getRawGyroYaw() -
          // Math.PI))
          // getPose.getRotation();
          // Calculate module setpoints
          ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(allianceSpeeds, 0.02);
          SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
          SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);
          Logger.recordOutput(
              "Swerve/Target Chassis Speeds Field Relative",
              ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, rawGyroRotation));
          Logger.recordOutput(
              "Swerve/Speed Error",
              discreteSpeeds.minus(getVelocity())); // weird coordinate system maybe?

          // Send setpoints to modules
          SwerveModuleState[] optimizedSetpointStates =
              Streams.zip(
                      Arrays.stream(modules),
                      Arrays.stream(setpointStates),
                      (m, s) -> m.runSetpoint(s))
                  .toArray(SwerveModuleState[]::new);

          // Log setpoint states
          Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
          Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
          Logger.recordOutput(
              "Setpoints Optimized Velocity Setpoint Module 1",
              optimizedSetpointStates[1].speedMetersPerSecond);
          Logger.recordOutput(
              "Measured Velocity Module 1", getModuleStates()[1].speedMetersPerSecond);
        });
  }

  public void setYaw(Rotation2d yaw) {
    gyroIO.setYaw(yaw);
    // poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), new
    // Pose2d(getPose().getTranslation(), yaw)); //this is alt fix if the other one doesnt work
    // gyroIO.setYaw(yaw);
    // poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), new
    // Pose2d(getPose().getTranslation(), yaw)); //this is alt fix if the other one doesnt work

    // does this need to be yaw instead of rawGyroRotation
  }
  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    // ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(driftCorrection(speeds), 0.02);
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    for (int i = 0; i < 4; i++) {
      if (Math.abs(setpointStates[i].speedMetersPerSecond) < 0.0001)
        setpointStates[i].angle = lastModulePositions[i].angle;
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    optimizedSetpointStates =
        Streams.zip(
                Arrays.stream(modules), Arrays.stream(setpointStates), (m, s) -> m.runSetpoint(s))
            .toArray(SwerveModuleState[]::new);
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /** Don't run this please */
  public void zeroGyro() {
    // poseEstimator.resetPosition(new Rotation2d(0.0), getModulePositions(),
    // poseEstimator.getEstimatedPosition());
    // gyroIO.setYaw(AllianceFlipUtil.apply(new Rotation2d(0.0)));
    poseEstimator.resetPosition(
        gyroInputs.yawPosition,
        getModulePositions(),
        new Pose2d(getPose().getTranslation(), new Rotation2d(0.0)));
    // gyroIO.setYaw(AllianceFlipUtil.apply(new Rotation2d(0.0)));
    // poseEstimator.resetPosition(gyroInputs.yawPosition, getModulePositions(), new
    // Pose2d(getPose().getTranslation(), new Rotation2d(0.0)));
  }
  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Runs forwards at the commanded voltage. */
  public Command runCharacterizationVoltsCmd(double volts) {
    return this.run(() -> Arrays.stream(modules).forEach((mod) -> mod.runCharacterization(volts)));
  }

  public static ModuleIO[] createTalonFXModules() {
    return new ModuleIO[] {
      new ModuleIOTalonFX(0), new ModuleIOTalonFX(1), new ModuleIOTalonFX(2), new ModuleIOTalonFX(3)
    };
  }

  public void setTurnState(SwerveModuleState[] states) {
    for (int i = 0; i < 4; i++) {
      states[i].speedMetersPerSecond = 0.0;
    }
    SwerveModuleState[] optimizedSetpointStates =
        Streams.zip(Arrays.stream(modules), Arrays.stream(states), (m, s) -> m.runSetpoint(s))
            .toArray(SwerveModuleState[]::new);
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(states[i]);
    }
  }
  /**
   * Constructs an array of swerve module ios corresponding to a simulated robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createSimModules() {
    return new ModuleIO[] {
      new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim()
    };
  }

  public static ModuleIO[] createModuleIOs() {
    return new ModuleIO[] {
      new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}
    };
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command slipCurrentTest() {
    return m_slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
  }
  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot with Gyro Rotation")
  public Pose2d getPoseGyroRot() {
    return new Pose2d(
        poseEstimator.getEstimatedPosition().getTranslation(),
        new Rotation2d(MathUtil.angleModulus(rawGyroRotation())));
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    // setYaw(pose.getRotation());
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return MAX_LINEAR_SPEED;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return MAX_ANGULAR_SPEED;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0),
      new Translation2d(-TRACK_WIDTH_X / 2.0, -TRACK_WIDTH_Y / 2.0)
    };
  }
  /** Get the position of all drive wheels in radians. */
  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules)
        .mapToDouble((module) -> module.getPosition().distanceMeters / WHEEL_RADIUS)
        .toArray();
  }
  /** Gets the raw, unwrapped gyro heading from the gyro. Useful for wheel characterization */
  public double getRawGyroYaw() {
    return gyroInputs.yawPosition.getRadians();
  }

  public double rawGyroRotation() {
    return rawGyroRotation.getDegrees();
  }

  private static double averageAmbiguity(PhotonPipelineResult x) {
    double sum = 0;
    sum = x.getTargets().stream().mapToDouble((target) -> target.getPoseAmbiguity()).sum();

    return sum / x.getTargets().size();
    // Stream.of(x.getTargets()).forEach((target) -> sum += target.getPoseAmbiguity());
  }

  public void setDriveCurrentLimit(double currentLimit) {
    for (Module mod : modules) {
      mod.setDriveCurrentLimit(currentLimit);
    }
  }

  public void shouldEnableVision(boolean enable) {
    visionInAutonomous = enable;
  }

  public ChassisSpeeds driftCorrection(ChassisSpeeds speeds) {
    double xy = Math.abs(speeds.vxMetersPerSecond) + Math.abs(speeds.vyMetersPerSecond);

    if (Math.abs(speeds.omegaRadiansPerSecond) > 0.0 || pXY <= 0)
      desiredHeading = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    else if (xy > 0)
      speeds.omegaRadiansPerSecond +=
          driftCorrectionPID.calculate(
              poseEstimator.getEstimatedPosition().getRotation().getDegrees(), desiredHeading);

    pXY = xy;
    return speeds;
  }

  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    ChassisSpeeds speeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            kinematics.toChassisSpeeds(
                Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new)),
            getRotation());
    return speeds;
  }
}
