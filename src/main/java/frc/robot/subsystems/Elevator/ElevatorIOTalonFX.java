package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {
    public TalonFX rightMotor = new TalonFX(Constants.ClimbConstants.CLIMB_RIGHT_MOTOR);
    public TalonFX leftMotor = new TalonFX(Constants.ClimbConstants.CLIMB_LEFT_MOTOR);

    public ElevatorIOTalonFX(){
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        inputs.elevatorPositionMeters =
    }
}
