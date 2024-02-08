package frc.robot.subsystems.Climb;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ClimbConstants;

public class ElevatorSubsystem extends SubsystemBase{
    //Device number and CAN ID can only be entered later
    ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    DigitalInput hallEffect = new DigitalInput(Constants.ClimbConstants.HALLEFFECT);
    public ElevatorFeedforward feedforward;
    // private WPI_TalonFX encoderRight;
    // private WPI_TalonFX encoderLeft;
    public static double elevatorSetPoint = 0; 
    public static boolean extended = false;
 

    private static DigitalInput limitSwitch;

    public ElevatorSubsystem(ElevatorIO io){
        this.io = io;
        if(Robot.isReal()){
            feedforward = new ElevatorFeedforward(ClimbConstants.kS, ClimbConstants.kG, ClimbConstants.kV, ClimbConstants.kA);
        }
        else{
            feedforward = new ElevatorFeedforward(ClimbConstants.Sim.kS, ClimbConstants.Sim.kG, ClimbConstants.Sim.kV, ClimbConstants.Sim.kA);
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

    public void setElevatorSetPoint(double input){
        elevatorSetPoint = input; 
    }

    public static boolean getExtended(){
        return extended;
    }

    //Extends the Elevator
    public void extendElevator(){
        while(getRightEncoderPos() <= elevatorSetPoint || getLeftEncoderPos() <= elevatorSetPoint){
            rightMotor.set(0.5); //placeholder value
            leftMotor.set(0.5); 
            extended = true; 
        }
    }

    public void retractElevator(){
        while(getRightEncoderPos() >= 0 || getLeftEncoderPos() >= 0 && !getLimitSwitchState()){
            rightMotor.set(-0.5); 
            leftMotor.set(-0.5); 
            extended = false; 
        }
    }
    

}
