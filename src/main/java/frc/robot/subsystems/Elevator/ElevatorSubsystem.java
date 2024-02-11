package frc.robot.subsystems.Elevator;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase{
    //Device number and CAN ID can only be entered later
    ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    DigitalInput hallEffect = new DigitalInput(Constants.ElevatorConstants.HALLEFFECT);

    private ElevatorFeedforward feedforward;
    private ExponentialProfile profile;
    private PIDController pid;

    private ExponentialProfile.State setpoint = new ExponentialProfile.State(0, 0);
    public static boolean extended = false;
 

    private static DigitalInput limitSwitch;

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
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        io.rightResetEncoder(); 
        io.leftResetEncoder(); 
    }
    
    public static boolean getLimitSwitchState() {
        return limitSwitch.get();
    }

    public double getRightEncoderPos(){
        return rightMotor.getRotorPosition().getValueAsDouble(); 
    }

    public double getLeftEncoderPos(){
        return leftMotor.getRotorPosition().getValueAsDouble(); 
    }

    public static boolean getExtended(){
        return extended;
    }

    //Extends the Elevator
    public void extendElevator(double goal){
        var goalState = new ExponentialProfile.State(goal, 0);

        var next = profile.calculate(0.020, setpoint, goalState);

        // With the setpoint value we run PID control like normal
        double pidOutput = pid.calculate(inputs.elevatorPositionMeters, setpoint.position);
        double feedforwardOutput = feedforward.calculate(setpoint.velocity, next.velocity, 0.020);

        io.setVoltage(pidOutput + feedforwardOutput);

        setpoint = next;
    }

    @Override
    public void simulationPeriodic(){
        Logger.processInputs(getName(),inputs);
        io.updateInputs(inputs);
    }
    @Override
    public void periodic(){
        Logger.processInputs(getName(),inputs);
        io.updateInputs(inputs);
    }
}
