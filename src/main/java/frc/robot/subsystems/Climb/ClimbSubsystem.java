package frc.robot.subsystems.Climb;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase{
    //Device number and CAN ID can only be entered later
    public TalonFX rightMotor = new TalonFX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR);
    public TalonFX leftMotor = new TalonFX(Constants.ClimbConstants.CLIMB_LEFT_MOTOR);
     DigitalInput hallEffect = new DigitalInput(Constants.ClimbConstants.HALLEFFECT);
    // private WPI_TalonFX encoderRight;
    // private WPI_TalonFX encoderLeft;
    public static double encoderSetPoint = 0; 
    public static boolean extended = false;
 

    private static DigitalInput limitSwitch;

    public ClimbSubsystem(){
        
        rightMotor.setNeutralMode(NeutralModeValue.Brake);
        leftMotor.setNeutralMode(NeutralModeValue.Brake);

        rightResetEncoder(); 
        leftResetEncoder(); 
    }
    
    public static boolean getLimitSwitchState() {
        return limitSwitch.get();
    }

    public double getRightEncoderPos(){
        return rightMotor.getSelectedSensorPosition(); 
    }

    public double  getLeftEncoderPos(){
        return leftMotor.getSelectedSensorPosition(); 
    }

    public void setElevatorSetPoint(double input){
        elevatorSetPoint = input; 
    }

    public static boolean getExtended(){
        return extended;
    }

    //Extends the Elevator
    public void extendElevator(){
        while(getRightEncoderPos() <= encoderSetPoint || getLeftEncoderPos() <= encoderSetPoint){
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
