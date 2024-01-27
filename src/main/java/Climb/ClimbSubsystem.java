package Climb;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class ClimbSubsystem {
    //Device number and CAN ID can only be entered later
    private TalonFX rightMotor; 
    private TalonFX leftMotor; 
    private WPI_TalonFX encoderRight;
    private WPI_TalonFX encodeLeft;
    public static double  encoderSetPoint = 0; 
    public static boolean extended = false;
 

    private static DigitalInput limitSwitch;

    public ClimbSubsystem(){
        rightMotor = new TalonFX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR);
        leftMotor =  new TalonFX(Constants.ClimbConstants.CLIMB_LEFT_MOTOR);
        rightHangMotor.setNeutralMode(NeutralMode.Brake);
        leftHangMotor.setNeutralMode(NeutralMode.Brake);
        encoderRight = new WPI_TalonFX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR);
        encoderLeft = new WPI_TalonFX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR);
        limitSwitch = new DigitalInput(Constants.ClimbConstants.CLIMB_HALL_EFFECT_DIGITAL_INPUT_PORT);

        rightResetEncoder(); 
        leftResetEncoder(); 
    }
    
    public static boolean getLimitSwitchState() {
        return limitSwitch.get();
    }

    public static double getRightEncoderPos(){
        return encoderRight.getSelectedSensorPosition(); 
    }

    public static double  getLeftEncoderPos(){
        return encoderLeft.getSelectedSensorPosition(); 
    }

    public void setElevatorSetPoint(double input){
        elevatorSetPoint = input; 
    }

    public static boolean getExtended(){
        return extended;
    }

    //Extends the Elevator
    public static void extendElevator(){
        while(getRightEncoderPos() <= encoderSetPoint || getLeftEncoderPos() <= encoderSetPoint){
            rightMotor.set(0.5); //placeholder value
            leftMotor.set(0.5); 
            extended = true; 
        }
    }

    public static void retractElevator(){
        while(getRightEncoderPos() >= 0 || getLeftEncoderPos() >= 0 && !getLimitSwitchState()){
            rightMotor.set(-0.5); 
            leftMotor.set(-0.5); 
            extended = false; 
        }
    }
    
    public static void setElevator(){
        //If the elevator is extended, retract it, and if the the elevator is retracted, extend it
        //eliminates the need for 2 buttons
        if(extended){
            retractElevator();
            return;
        }
        extendElevator();
    }


}
