package frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot;

public class ElevatorSubsystem extends SubsystemBase{
    //Device number and CAN ID can only be entered later
    ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    // private ElevatorVisualizer visualizer= new ElevatorVisualizer("ElevatorVisualizer", null);

    private ElevatorFeedforward feedforward;
    private ExponentialProfile profile;
    private PIDController pid;

    private ExponentialProfile.State setpoint = new ExponentialProfile.State(0, 0);

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
        if(Robot.isReal()){
            feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
            profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
                ElevatorConstants.maxV, ElevatorConstants.kV, ElevatorConstants.kA));
            pid = new PIDController(ElevatorConstants.kP, 0.0, ElevatorConstants.kD);
            
        }
        else if(Robot.isSimulation()){
            feedforward = new ElevatorFeedforward(ElevatorConstants.Sim.kS, ElevatorConstants.Sim.kG, ElevatorConstants.Sim.kV, ElevatorConstants.Sim.kA);
            profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
                ElevatorConstants.maxV, ElevatorConstants.Sim.kV, ElevatorConstants.Sim.kA));
            pid = new PIDController(ElevatorConstants.Sim.kP, 0.0, ElevatorConstants.Sim.kD);
        }
        else{
            feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
            profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(
                ElevatorConstants.maxV, ElevatorConstants.kV, ElevatorConstants.kA));
            pid = new PIDController(ElevatorConstants.kP, 0.0, ElevatorConstants.kD);
        }

        io.resetLeftEncoder(); 
        io.resetRightEncoder(); 
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
        goal = MathUtil.clamp(goal, 0.0, ElevatorConstants.SOFT_LIMIT_HEIGHT);
        setpoint = new ExponentialProfile.State(getAverageExtension(), getAverageVelocity());
        var goalState = new ExponentialProfile.State(goal, 0);

        var next = profile.calculate(0.020, setpoint, goalState);

        // With the setpoint value we run PID control like normal
        double pidOutput1 = pid.calculate(inputs.carriagePositionMeters[0], goal);
        double pidOutput2 = pid.calculate(inputs.carriagePositionMeters[1], goal);
        double feedforwardOutput = feedforward.calculate(setpoint.velocity, next.velocity, 0.020);


        setVoltage(pidOutput1 + feedforwardOutput, pidOutput2 + feedforwardOutput);
    }

    @Override
    public void simulationPeriodic(){
        Logger.processInputs(getName(),inputs);
        io.updateInputs(inputs);
        if(getHallEffectState()){
            io.resetLeftEncoder();
            io.resetRightEncoder();
        }
        // visualizer.updateSim(getAverageExtension());
    }
    @Override
    public void periodic(){
        Logger.processInputs(getName(),inputs);
        io.updateInputs(inputs);
         if(getHallEffectState()){
            io.resetLeftEncoder();
            io.resetRightEncoder();
        }
        // visualizer.updateSim(getAverageExtension());
    }
}