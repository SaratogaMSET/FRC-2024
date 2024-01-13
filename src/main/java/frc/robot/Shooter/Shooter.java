package frc.robot.Shooter;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
public class Shooter extends SubsystemBase {
    private TalonFX shooterMotor1 = new TalonFX(26);
    private TalonFX shooterMotor2 = new TalonFX(50);

    public void run(double speed1, double speed2){
        shooterMotor1.set(speed1);
        shooterMotor2.set(speed2);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Shooter Motor 1", shooterMotor1.getRotorVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter Motor 2", shooterMotor2.getRotorVelocity().getValueAsDouble());
    }
}
