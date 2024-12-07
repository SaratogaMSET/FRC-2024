package frc.robot.subsystems.Superstructure.Elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;
import frc.robot.Robot;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  // Device number and CAN ID can only be entered later
  ElevatorIO io;
  private final Servo leftFlipOut =
      new Servo(Elevator.LEFTSERVO_CHANNEL); // TODO: CHANGE CONSTANTSs
  private final Servo rightFlipOut =
      new Servo(Elevator.RIGHTSERVO_CHANNEL); // TODO: CHANGE CONSTANTSs
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  // private ElevatorVisualizer visualizer= new ElevatorVisualizer("ElevatorVisualizer", null);
  private final SysIdRoutine sysId;
  private ElevatorFeedforward feedforward;
  // private ExponentialProfile profile;
  private PIDController pid;

  private ExponentialProfile.State setpoint = new ExponentialProfile.State(0, 0);

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
    if (Robot.isReal()) {
      feedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV, Elevator.kA);
      // profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
      // Elevator.maxV, Elevator.kV, Elevator.kA));
      pid = new PIDController(Elevator.kP, 0.0, Elevator.kD);

    } else if (Robot.isSimulation()) {
      feedforward =
          new ElevatorFeedforward(
              Elevator.Sim.kS, Elevator.Sim.kG, Elevator.Sim.kV, Elevator.Sim.kA);
      // profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
      //     Elevator.maxV, Elevator.Sim.kV, Elevator.Sim.kA));
      pid = new PIDController(Elevator.Sim.kP, 0.0, Elevator.Sim.kD);
    } else {
      feedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV, Elevator.kA);
      // profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
      //     Elevator.maxV, Elevator.kV, Elevator.kA));
      pid = new PIDController(Elevator.kP, 0.0, Elevator.kD);
    }

    io.resetLeftEncoder();
    io.resetRightEncoder();
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  setVoltage(voltage.in(Volts), voltage.in(Volts));
                },
                null,
                this));
  }

  public boolean getHallEffectState() {
    return inputs.hallEffectTriggered;
  }

  public double getAverageExtension() {
    return (inputs.carriagePositionMeters[0] + inputs.carriagePositionMeters[1]) / 2;
  }

  public double getAverageVelocity() {
    return (inputs.elevatorVelocityMetersPerSec[0] + inputs.elevatorVelocityMetersPerSec[1]) / 2;
  }

  public double getSecondStageLength() {
    return inputs.secondStagePositionMeters;
  }

  public void setVoltage(double voltage1, double voltage2) {
    if (Robot.isReal()) {
      if ((getAverageExtension() >= Elevator.SOFT_LIMIT_HEIGHT && voltage1 > 0)
          || (getAverageExtension() <= 0.0 && voltage1 < 0)) {
        io.leftSetVoltage(0.0);
        io.rightSetVoltage(0.0);
      } else {
        io.leftSetVoltage(voltage1);
        io.rightSetVoltage(voltage2);
      }
    } else {
      io.setVoltage(voltage1);
    }
  }

  public void resetEncoders() {
    io.resetLeftEncoder();
    io.resetRightEncoder();
  }
  // Extends the Elevator
  public void setSetpoint(double goal) {
    goal = MathUtil.clamp(goal, 0.0, Elevator.SOFT_LIMIT_HEIGHT);

    // setpoint = new ExponentialProfile.State(getAverageExtension(), getAverageVelocity());
    // var goalState = new ExponentialProfile.State(goal, 0);

    // var next = profile.calculate(0.020, setpoint, goalState);

    // With the setpoint value we run PID control like normal
    double pidOutput1 = pid.calculate(inputs.carriagePositionMeters[0], goal);
    double pidOutput2 = pid.calculate(inputs.carriagePositionMeters[1], goal);
    double ff = Math.signum(goal) * Elevator.kG;

    setVoltage(pidOutput1 + ff, pidOutput2 + ff);
    // setVoltage(pidOutput1, pidOutput2);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public Command flipIn() {
    return this.runOnce(() -> leftFlipOut.setAngle(0.0))
        .alongWith(this.runOnce(() -> rightFlipOut.setAngle(0.0)));
  }

  public Command flipOut() {
    return this.runOnce(() -> leftFlipOut.setAngle(50.0))
        .alongWith(this.runOnce(() -> rightFlipOut.setAngle(50.0)));
  }

  @AutoLogOutput private Goal goal = Goal.STOWED;

  public void setGoal(Goal desiredGoal) {
    goal = desiredGoal;
  }

  public enum Goal {
    SHOOTING(0),
    AMP(Constants.Intake.DesiredStates.Amp.elevatorPosition),
    GROUND_INTAKE(0),
    STOWED(0),
    HANG_UP(Elevator.ClimbHeight),
    HANG_DOWN(0),
    EXTAKING(0),
    TRAP(0),
    RESET_TRAP,
    SOURCE(0),
    RESET_WRIST(0);

    private final double[] positions;

    Goal() {
      this.positions = new double[0];
    }

    Goal(double... positions) {
      this.positions = positions;
    }

    public double[] getPositions() {
      return positions;
    }
  }

  @Override
  public void simulationPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    // if(getHallEffectState()){
    //     io.resetLeftEncoder();
    //     io.resetRightEncoder();
    // }

    // visualizer.updateSim(getAverageExtension());
  }

  private double elevatorGoalPosition;

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
    //  if(getHallEffectState()){
    //     io.resetLeftEncoder();
    //     io.resetRightEncoder();
    // }

    elevatorGoalPosition = goal.getPositions()[0];

    if (!atGoal()) {
      setSetpoint(elevatorGoalPosition);
    }

    SmartDashboard.putBoolean("Elevator Hall Effect", getHallEffectState());
    // visualizer.updateSim(getAverageExtension());
  }

  public boolean atGoal() {
    double currentPosition =
        (inputs.carriagePositionMeters[0] + inputs.carriagePositionMeters[1]) / 2;
    double position = currentPosition - elevatorGoalPosition;
    if (Math.abs(position) <= 1e-3) {
      return true;
    }
    return false;
  }
}
