package frc.robot.subsystems.Superstructure.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.DesiredStates.Amp;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.Constants.Intake.DesiredStates.Source;
import frc.robot.Constants.Intake.DesiredStates.Trap;
import frc.robot.Constants.Intake.Shoulder;
import frc.robot.Constants.Intake.Wrist;
import frc.robot.subsystems.Superstructure.Intake.Shoulder.ShoulderIO;
import frc.robot.subsystems.Superstructure.Intake.Shoulder.ShoulderIOInputsAutoLogged;
import frc.robot.subsystems.Superstructure.Intake.Wrist.WristIO;
import frc.robot.subsystems.Superstructure.Intake.Wrist.WristIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {
  public ShoulderIO shoulder;
  // public CANSparkMax WristMotorSus = new CANSparkMax(Wrist.MOTOR, MotorType.kBrushless);
  public WristIO wrist;
  // public RollerIO roller;
  ShoulderIOInputsAutoLogged shoulderIOInputs = new ShoulderIOInputsAutoLogged();
  public WristIOInputsAutoLogged wristIOInputs = new WristIOInputsAutoLogged();

  PIDController wristPID = new PIDController(Wrist.k_P, 0.0, Wrist.k_D);
  PIDController shoulderPID = new PIDController(Shoulder.k_P, 0.0, Shoulder.k_D);
  boolean previousHallEffect = false;

  public IntakeSubsystem(ShoulderIO shoulder, WristIO wrist) {
    this.shoulder = shoulder;
    this.wrist = wrist;
  }

  public double shoulderGetRads() {
    return shoulderIOInputs.shoulderRads;
  }

  public double shoulderGetRadPerSec() {
    return shoulderIOInputs.shoulderRadPerSecs;
  }

  public void setShoulderVoltage(double voltage) {
    if (voltage < 0 && shoulderGetRads() * 180 / Math.PI < -90) voltage = 0;
    if (voltage > 0 && shoulderGetRads() * 180 / Math.PI > 90) voltage = 0;
    shoulder.setVoltage(voltage);
    Logger.recordOutput("Intake/Shoulder/Voltage", voltage);
  }

  public double wristGetRads() {
    return wristIOInputs.wristRads;
  }

  public double wristGetRadPerSec() {
    return wristIOInputs.wristRadsPerSec;
  }

  public double motorShoulderRads() {
    return shoulderIOInputs.motorShoulderRads;
  }

  public boolean getCurrentLimitTripped() {
    return wrist.getCurrentLimitTripped();
  }

  public void setWristVoltage(double voltage) {
    // if(wristGetRads() > Wrist.HIGH_BOUND && voltage > 0) voltage = 0;
    // if(wrist.getHallEffect() && voltage < 0) voltage = 0;
    Logger.recordOutput("Intake/Wrist/VoltageReqSub", voltage);
    // wrist.setVoltage(voltage);
    wrist.motor.setVoltage(voltage);
    Logger.recordOutput("Intake/Wrist/Voltage", voltage);
  }

  public void setVoltages(double shoulderVoltage, double wristVoltage) {
    setShoulderVoltage(shoulderVoltage);
    setWristVoltage(wristVoltage);
  }
  // public void setRollerVoltage(double voltage){
  //     roller.setVoltage(voltage);
  //     Logger.recordOutput("Intake/Roller/Voltage", voltage);
  // }

  public Command setGravityCompensation(double additionalVoltage) {
    double voltageFF = Math.cos(shoulderGetRads() - 0.14) * Shoulder.k_G + additionalVoltage;
    return this.run(() -> setShoulderVoltage(voltageFF));
  }

  public void setAngleWrist(double angle) {
    // Enforce bounds on angle
    // angle = MathUtil.clamp(angle, Wrist.LOW_BOUND, Wrist.HIGH_BOUND);

    double voltageFB = wristPID.calculate(wristGetRads(), angle);

    setWristVoltage(voltageFB);

    Logger.recordOutput("Intake/Wrist/Angle Setpoint", angle);
    Logger.recordOutput("Intake/Wrist/Setpoint Voltage", voltageFB);
    Logger.recordOutput("Intake/Wrist/Current Angle", wristGetRads());
  }

  public void setAngleShoulder(double angle) {
    Logger.recordOutput("Intake/Shoulder/Setpoint", angle);
    double shoulderRads = shoulderGetRads();
    Logger.recordOutput("Intake/Shoulder/Delta", shoulderRads - angle);

    // angle = MathUtil.clamp(angle, Shoulder.HIGH_BOUND, Shoulder.LOW_BOUND);
    // Enforce bounds on angle
    angle = MathUtil.clamp(angle, Shoulder.LOW_BOUND, Shoulder.HIGH_BOUND);

    double voltageFB = MathUtil.clamp(shoulderPID.calculate(shoulderRads, angle), -4, 4);
    Logger.recordOutput(
        "Intake/Shoulder/Setpointdfddf", shoulderPID.calculate(shoulderRads, angle));
    // double maxVoltFromVelocity = shoulderGetRadPerSec() * 1.0 + 0.5;
    // if(Math.abs(voltageFB) > maxVoltFromVelocity) voltageFB = Math.signum(voltageFB) *
    // maxVoltFromVelocity;
    double voltageFF = Math.cos(shoulderRads - 0.14) * Shoulder.k_G;
    setShoulderVoltage(voltageFB + voltageFF);
  }

  public void setAngleShoulderMotionMagic(double angle) {
    /* Angle is target angle in radians for the shoulder! The Rotation to Radian conversion is in the IOReal! */

    Logger.recordOutput("Intake/Shoulder/Setpoint", angle);
    Logger.recordOutput(
        "RealOutputs/Intake/Shoulder/ActualRotationMotionMagic",
        Units.radiansToRotations(shoulderGetRads()));
    // double shoulderRads = shoulderGetRads();

    // double voltageFF = Math.cos(shoulderRads- 0.14) * Shoulder.k_G;
    shoulder.setProfiled(angle, 0.0);
  }

  public void setPreviousZeroed(boolean zeroed) {
    this.wristIOInputs.previouslyZeroed = zeroed;
  }

  public boolean getPreviousZeroed() {
    return this.wristIOInputs.previouslyZeroed;
  }

  public double getWristEncoderPosition() {
    return this.wristIOInputs.wristRotations;
  }

  public void setWristEncoderPosition(double angleRotations) {
    this.wrist.setWristPosition(angleRotations);
  }

  public static enum Goal {
    SHOOTING(Neutral.shoulderAvoidTurretAngle, Neutral.wristAvoidTurretAngle),
    AMP(Amp.SHOULDER_ANGLE, Amp.WRIST_ANGLE),
    GROUND_INTAKE(Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE),
    STOWED(Neutral.SHOULDER_ANGLE, Neutral.WRIST_ANGLE),
    HANG_UP(0, 0),
    HANG_DOWN(0, 0),
    EXTAKING(Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE),
    SOURCE(Source.SHOULDER_ANGLE, Source.WRIST_ANGLE),
    TRAP(Trap.SHOULDER_ANGLE, Trap.WRIST_ANGLE),
    RESET_TRAP(0, 0),
    RESET_WRIST(0, Neutral.DISABLED_WRIST);

    private final double[] angles;

    Goal() {
      this.angles = new double[0];
    }

    Goal(double... angles) {
      this.angles = angles;
    }

    public double[] getAngles() {
      return angles;
    }
  }

  @Override
  public void simulationPeriodic() {
    shoulder.updateInputs(shoulderIOInputs);
    wrist.updateInputs(wristIOInputs);
    Logger.processInputs(getName(), shoulderIOInputs);
    Logger.processInputs(getName(), wristIOInputs);
  }

  private double wristGoalAngle;
  private double shoulderGoalAngle;

  @AutoLogOutput private Goal goal = Goal.STOWED;

  public void setGoal(Goal desiredGoal) {
    goal = desiredGoal;
  }

  @Override
  public void periodic() {
    shoulder.updateInputs(shoulderIOInputs);
    wrist.updateInputs(wristIOInputs);
    // if(wrist.getHallEffect()) wrist.manualHallEffectReset();
    // roller.updateInputs(rollerIOInputs);
    // wrist.hallEffectReset();

    shoulderGoalAngle = goal.getAngles()[0];
    wristGoalAngle = goal.getAngles()[1];

    Logger.recordOutput("Intake/GoalWrist", wristGoalAngle);

    if (!atGoal()) {
      setAngleShoulderMotionMagic(shoulderGoalAngle);
      Logger.recordOutput("Intake/GoalShoulder", shoulderGoalAngle);
      setAngleWrist(wristGoalAngle);
    }

    Logger.recordOutput("Intake/Shoulder/Angle", shoulderGetRads());
    Logger.recordOutput(
        "Intake/Shoulder/FF", Math.cos(shoulderGetRads()) * Shoulder.k_G); // Thanks andrew :D

    Logger.processInputs(getName(), shoulderIOInputs);
    Logger.processInputs(getName(), wristIOInputs);
    // Logger.processInputs(getName(), rollerIOInputs);
    // runArm();
    // viz.updateSim(shoulderIOInputs.shoulderDegrees, wristIOInputs.wristDegrees);
  }

  @AutoLogOutput(key = "Superstructure/Arm/shoulderAtGoal")
  public boolean shoulderAtGoal() {
    double angle = shoulderGetRads() - shoulderGoalAngle;
    if (Math.abs(angle) <= 0.1) {
      return true;
    }
    return false;
  }

  @AutoLogOutput(key = "Superstructure/Arm/wristAtGoal")
  public boolean wristAtGoal() {
    double angle = wristGetRads() - wristGoalAngle;
    if (Math.abs(angle) <= 0.1) {
      return true;
    }
    return false;
  }

  @AutoLogOutput(key = "Superstructure/Arm/AtGoal")
  public boolean atGoal() {
    if (shoulderAtGoal() && wristAtGoal()) {
      return true;
    }
    return false;
  }
}
