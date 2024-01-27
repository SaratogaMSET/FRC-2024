// Copyright 2021-2023 FRC 6328
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

package frc.robot.Subsystems.Swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.google.common.collect.Streams;
import com.google.flatbuffers.FlexBuffers.Vector;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Subsystems.Swerve.Module.ModuleConstants;
import frc.robot.Subsystems.Vision.VisionIO;
import frc.robot.Subsystems.Vision.VisionIO.VisionIOInputs;

import frc.robot.Subsystems.Swerve.GyroIO.GyroIOInputsAutoLogged;

public class SwerveSubsystem extends SubsystemBase {
  // Drivebase constants
  public static final double MAX_LINEAR_SPEED = Units.feetToMeters(14.5); //TODO: Fix
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(25.0); //TODO: Fix
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0); //TODO: Fix
  public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;
  // Hardware constants
  public static final int PigeonID = 15;

  public static final ModuleConstants frontLeft =
      new ModuleConstants("Front Left", 2, 3, 4, Rotation2d.fromDegrees(0.0));
  public static final ModuleConstants frontRight =
      new ModuleConstants("Front Right", 5, 6, 8, Rotation2d.fromDegrees(0.0));
  public static final ModuleConstants backLeft =
      new ModuleConstants("Back Left", 9, 10, 11, Rotation2d.fromDegrees(0.0));
  public static final ModuleConstants backRight =
      new ModuleConstants("Back Right", 12, 13, 14, Rotation2d.fromDegrees(0.0));

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());

  private final VisionIO visionIO;
  private VisionIOInputs vision_inputs;

  private SwerveDrivePoseEstimator m_PoseEstimator;
  private SwerveDriveOdometry m_Odometry;

  private Pose2d targetPose = new Pose2d();
  private List<Pose2d> activePath = new ArrayList<Pose2d>();
  // private Pose2d pose = new Pose2d();
  private Rotation2d lastGyroRotation = new Rotation2d();
  
  private Supplier<Matrix<N3, N1>> stdDevsSupplier;
  private boolean seeded = false;

  public SwerveSubsystem(VisionIO visionIO, GyroIO gyroIO, ModuleIO... moduleIOs) {
    this.gyroIO = gyroIO;

    this.visionIO = visionIO;
    vision_inputs = new VisionIOInputs();
    
    modules = new Module[moduleIOs.length];

    for (int i = 0; i < moduleIOs.length; i++) {
      modules[i] = new Module(moduleIOs[i]);
    }

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::setPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::runVelocity, // Method that will drive the robot given ROBOT RELATIVE
        // ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  var alliance = DriverStation.getAlliance();
                  if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Red;
                  }
                  return false;
              },
        this // Reference to this subsystem to set requirements
        );
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> Logger.recordOutput("PathPlanner/Target", targetPose));

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> Logger.recordOutput("PathPlanner/ActivePath", activePath.toString().getBytes()));

    m_PoseEstimator = new SwerveDrivePoseEstimator(kinematics,
      getRotation2d(), 
      getModulePositions(), 
      new Pose2d(),
      // pose, 
      Constants.Vision.stateSTD, 
      Constants.Vision.visDataSTD); 

    m_Odometry = new SwerveDriveOdometry(kinematics, getRotation2d(), getModulePositions());

  }

  /**
   * Constructs an array of swerve module ios corresponding to the real robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createTalonFXModules() {
    return new ModuleIO[] {
      new ModuleIOTalonFX(frontLeft),
      new ModuleIOTalonFX(frontRight),
      new ModuleIOTalonFX(backLeft),
      new ModuleIOTalonFX(backRight)
    };
  }

  /**
   * Constructs an array of swerve module ios corresponding to a simulated robot.
   *
   * @return The array of swerve module ios.
   */
  public static ModuleIO[] createSimModules() {
    return new ModuleIO[] {
      new ModuleIOSim("FrontLeft"),
      new ModuleIOSim("FrontRight"),
      new ModuleIOSim("BackLeft"),
      new ModuleIOSim("BackRight")
    };
  }

  public static ModuleIO[] createModuleIOs() {
    return new ModuleIO[] {
      new ModuleIO() {},
      new ModuleIO() {},
      new ModuleIO() {},
      new ModuleIO() {}
    };
  }

  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Swerve/Gyro", gyroInputs);
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

    // Update odometry
    int deltaCount =
        Math.min(
            gyroInputs.connected ? gyroInputs.odometryYawPositions.length : Integer.MAX_VALUE,
            Arrays.stream(modules)
                .map((m) -> m.getPositionDeltas().length)
                .min(Integer::compare)
                .get());
    for (int deltaIndex = 0; deltaIndex < deltaCount; deltaIndex++) {
      // Read wheel deltas from each module
      SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        wheelDeltas[moduleIndex] = modules[moduleIndex].getPositionDeltas()[deltaIndex];
      }

      // The twist represents the motion of the robot since the last
      // sample in x, y, and theta based only on the modules, without
      // the gyro. The gyro is always disconnected in simulation.
      var twist = kinematics.toTwist2d(wheelDeltas);
      if (gyroInputs.connected) {
        // If the gyro is connected, replace the theta component of the twist
        // with the change in angle since the last sample.
        Rotation2d gyroRotation = gyroInputs.odometryYawPositions[deltaIndex];
        twist = new Twist2d(twist.dx, twist.dy, gyroRotation.minus(lastGyroRotation).getRadians());
        lastGyroRotation = gyroRotation;
      }
      // Apply the twist (change since last sample) to the current pose
      // pose = pose.exp(twist);
      m_PoseEstimator.update(lastGyroRotation, getModulePositions());
      m_Odometry.update(lastGyroRotation, getModulePositions());
    }

    visionIO.updateInputs(vision_inputs, new Pose3d(getOdomPose()));
    var visionData = vision_inputs.estPose;
    var timestamp = vision_inputs.timestamp;

    if (!visionData.isPresent()) return;
    var inst_pose = visionData.get().estimatedPose.toPose2d();
    if (seeded == false){
      seeded = true;
      m_PoseEstimator.resetPosition(getRotation(), getModulePositions(), inst_pose);
      m_Odometry.resetPosition(getRotation(), getModulePositions(), inst_pose);
      SmartDashboard.putNumberArray("Seed Pose", new double[] {inst_pose.getTranslation().getX(), inst_pose.getTranslation().getY()});
      
    } else if (DriverStation.isTeleop() && getPose().getTranslation().getDistance(inst_pose.getTranslation()) < 10){
        m_PoseEstimator.addVisionMeasurement(inst_pose, timestamp);
        // m_PoseEstimator.addVisionMeasurement(inst_pose, timestamp, stdDevsSupplier.get()); TODO: BRING ME BACK
        SmartDashboard.putNumberArray("Vision Poses", new double[]{inst_pose.getTranslation().getX(), inst_pose.getTranslation().getY()});
    }

    Logger.recordOutput("Swerve Odometry Pose", m_Odometry.getPoseMeters());
    Logger.recordOutput("Swerve UKF Pose", m_PoseEstimator.getEstimatedPosition());
    // Logger.recordOutput("Speakesr Delta Rotation Degrees", getSpeakerRotationDelta().getDegrees());
  }

  private void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, MAX_LINEAR_SPEED);

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates =
        Streams.zip(
                Arrays.stream(modules), Arrays.stream(setpointStates), (m, s) -> m.runSetpoint(s))
            .toArray(SwerveModuleState[]::new);

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public Command runVelocityCmd(Supplier<ChassisSpeeds> speeds) {
    return this.run(() -> runVelocity(speeds.get()));
  }

  /** Stops the drive. */
  public Command stopCmd() {
    return runVelocityCmd(ChassisSpeeds::new);
  }

  public Command runVelocityFieldRelative(Supplier<ChassisSpeeds> speeds) {
    return this.runVelocityCmd(
        () -> ChassisSpeeds.fromFieldRelativeSpeeds(speeds.get(), getRotation()));
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public Command stopWithXCmd() {
    return this.run(
        () -> {
          Rotation2d[] headings = new Rotation2d[4];
          for (int i = 0; i < modules.length; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
          }
          kinematics.resetHeadings(headings);
          stopCmd();
        });
  }

  /** Runs forwards at the commanded voltage. */
  public Command runCharacterizationVoltsCmd(double volts) {
    return this.run(() -> Arrays.stream(modules).forEach((mod) -> mod.runCharacterization(volts)));
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocitoes) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {

    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }

    return states;
  }

  @AutoLogOutput(key = "Odometry/Velocity")
  public ChassisSpeeds getVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        kinematics.toChassisSpeeds(
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new)),
        getRotation());
  }

  @AutoLogOutput(key = "Odometry/RobotRelativeVelocity")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        (SwerveModuleState[])
            Arrays.stream(modules).map((m) -> m.getState()).toArray(SwerveModuleState[]::new));
  }

  /** Returns the current estimated pose. */
  @AutoLogOutput(key = "Odometry/EstimatorPose")
  public Pose2d getPose() {
    return m_PoseEstimator.getEstimatedPosition();
  }

   /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/OdometryPose")
  public Pose2d getOdomPose() {
    return m_Odometry.getPoseMeters();
  }

  /** Returns the current estimated rotation. */
  public Rotation2d getRotation() {
    return m_PoseEstimator == null ? new Pose2d().getRotation() : m_PoseEstimator.getEstimatedPosition().getRotation();
  }

  public Rotation2d getRotation2d(){
    if (gyroInputs.connected) {
      // If the gyro is connected, replace the theta component of the twist
      // with the change in angle since the last sample.
      return gyroInputs.yawPosition;
    } else {
      // what now?
      return getRotation();
    }
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    this.m_PoseEstimator.resetPosition(lastGyroRotation, getModulePositions(), pose);
    this.m_Odometry.resetPosition(lastGyroRotation, getModulePositions(), pose);
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

  public SwerveModulePosition[] getModulePositions(){
    return (Arrays.stream(modules).map(this_module -> this_module.getPosition())).toArray(SwerveModulePosition[]::new);
  }

  // public Rotation2d getSpeakerRotationDelta(){
    /** Returns Rotation 2d object for shooter implementation. Negative Result means the turret needs to turn left. */
    // Pose2d thisPose = m_PoseEstimator.getEstimatedPosition();
    // Translation2d deltaPose = thisPose.minus(FieldConstants.Speaker.centerSpeakerOpening).getTranslation();

    // // Get the Unit vector of robot rotation. 
    // Translation2d robotRotationUnitTranslation = new Translation2d(thisPose.getRotation().getCos(), thisPose.getRotation().getSin()); 

    // edu.wpi.first.math.Vector<N2> deltaPoseVector = VecBuilder.fill(deltaPose.getX(), deltaPose.getY());
    // edu.wpi.first.math.Vector<N2> robotRotationUnitVector = VecBuilder.fill(robotRotationUnitTranslation.getX(), robotRotationUnitTranslation.getY());
    
    // // Calculate the angle between the turret's forward vector and the target vector
    // double angleToTarget = Math.acos(robotRotationUnitVector.dot(deltaPoseVector) /
    //                                 (robotRotationUnitVector.norm() * deltaPoseVector.norm()));

    // if(thisPose.getY() > FieldConstants.Speaker.centerSpeakerOpening.getY()) return Rotation2d.fromRadians(-angleToTarget);   
    // return Rotation2d.fromRadians(angleToTarget);

    // return new Rotation2d(Math.atan2(deltaPose.getY(), deltaPose.getX()));
    
    // return thisPose.minus(FieldConstants.Speaker.centerSpeakerOpening).getRotation();
  // }
}