package frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Elevator;
import frc.robot.Robot;
import static edu.wpi.first.units.Units.Volts;

public class ElevatorSubsystem extends SubsystemBase{
    //Device number and CAN ID can only be entered later
    ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    // private ElevatorVisualizer visualizer= new ElevatorVisualizer("ElevatorVisualizer", null);
    private final SysIdRoutine sysId;
    // private ElevatorFeedforward feedforward;
    // private ExponentialProfile profile;
    private PIDController pid;

    private ExponentialProfile.State setpoint = new ExponentialProfile.State(0, 0);

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
        if(Robot.isReal()){
            // feedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV, Elevator.kA);
            // profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
                // Elevator.maxV, Elevator.kV, Elevator.kA));
            pid = new PIDController(Elevator.kP, 0.0, Elevator.kD);
            
        }
        else if(Robot.isSimulation()){
            // feedforward = new ElevatorFeedforward(Elevator.Sim.kS, Elevator.Sim.kG, Elevator.Sim.kV, Elevator.Sim.kA);
            // profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
            //     Elevator.maxV, Elevator.Sim.kV, Elevator.Sim.kA));
            pid = new PIDController(Elevator.Sim.kP, 0.0, Elevator.Sim.kD);
        }
        else{
            // feedforward = new ElevatorFeedforward(Elevator.kS, Elevator.kG, Elevator.kV, Elevator.kA);
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
    public double getAverageExtension(){
        return (inputs.carriagePositionMeters[0] + inputs.carriagePositionMeters[1])/2;
    }
     public double getAverageVelocity(){
        return (inputs.elevatorVelocityMetersPerSec[0] + inputs.elevatorVelocityMetersPerSec[1])/2;
    }
    public double getSecondStageLength(){
        return inputs.secondStagePositionMeters;
    }
    public void setVoltage(double voltage1, double voltage2){
        if(Robot.isReal()){
            io.leftSetVoltage(voltage1);
            io.rightSetVoltage(voltage2);
        }
        else{
            io.setVoltage(voltage1);
        }
    }

    //Extends the Elevator
    public void setSetpoint(double goal){
        goal = MathUtil.clamp(goal, 0.0, Elevator.SOFT_LIMIT_HEIGHT);
        setpoint = new ExponentialProfile.State(getAverageExtension(), getAverageVelocity());
        var goalState = new ExponentialProfile.State(goal, 0);

        // var next = profile.calculate(0.020, setpoint, goalState);

        // With the setpoint value we run PID control like normal
        double pidOutput1 = pid.calculate(inputs.carriagePositionMeters[0], goal);
        double pidOutput2 = pid.calculate(inputs.carriagePositionMeters[1], goal);
        // double feedforwardOutput = feedforward.calculate(setpoint.velocity, next.velocity, 0.020);


        // setVoltage(pidOutput1 + feedforwardOutput, pidOutput2 + feedforwardOutput);
        setVoltage(pidOutput1, pidOutput2);
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
  
    @Override
    public void simulationPeriodic(){
        io.updateInputs(inputs);
        Logger.processInputs(getName(),inputs);
        if(getHallEffectState()){
            io.resetLeftEncoder();
            io.resetRightEncoder();
        }
        // visualizer.updateSim(getAverageExtension());
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(getName(),inputs);
         if(getHallEffectState()){
            io.resetLeftEncoder();
            io.resetRightEncoder();
        }
        SmartDashboard.putBoolean("Elevator Hall Effect", getHallEffectState());
        // visualizer.updateSim(getAverageExtension());
    }
}
