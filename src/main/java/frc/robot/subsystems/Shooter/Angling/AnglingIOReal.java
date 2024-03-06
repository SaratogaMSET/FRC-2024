package frc.robot.subsystems.Shooter.Angling;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.Constants.ShooterPivotConstants;

public class AnglingIOReal implements AnglingIO{
     TalonFX angleMotor = new TalonFX(ShooterPivotConstants.kMotorPort, Constants.CANBus);
    CANcoder encoder = new CANcoder(ShooterPivotConstants.kEncoderPort, Constants.CANBus);

    public AnglingIOReal(){
        angleMotor.setInverted(true);
        angleMotor.setControl(new StaticBrake());
    }

    @Override
    public void updateInputs(AnglingIOInputs inputs){
        inputs.pivotAppliedVolts = angleMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotAppliedCurrent = angleMotor.getStatorCurrent().getValueAsDouble();
        inputs.pivotRadPerSec = angleMotor.getVelocity().getValueAsDouble() * 2 * Math.PI / ShooterPivotConstants.kMotorGearing;
        inputs.pivotRad = 2 * Math.PI * (-encoder.getAbsolutePosition().getValueAsDouble() - ShooterPivotConstants.kEncoderOffset);
    }


    @Override
    public void setPivotVoltage(double voltage){
        angleMotor.setVoltage(voltage);
    }
}
