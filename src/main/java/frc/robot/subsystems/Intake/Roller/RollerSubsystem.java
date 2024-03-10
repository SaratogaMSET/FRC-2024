package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase{

    public RollerIO io;
    RollerIOInputsAutoLogged rollerIOInputs = new RollerIOInputsAutoLogged();

    public RollerSubsystem(RollerIO rollerIO){
        this.io = rollerIO;
    }
    
    public boolean getIntakeBeamBreak(){
        return rollerIOInputs.intakeIR;
    }

     public boolean getShooterBeamBreak(){
        return rollerIOInputs.shooterIR;
    }
    
    public void setIntakeFeederVoltage(double voltage){
        io.setIntakeFeederVoltage(voltage);
    }

    public void setShooterFeederVoltage(double voltage){
        io.setShooterFeederVoltage(voltage);
    }

     public void setIntakeFeederMode(boolean brake){
    if(brake){
      io.setIntakeFeederMode(brake);
    }
    else{
      io.setIntakeFeederMode(brake);
    }
  }

  public void setShooterFeederMode(boolean brake){
    if(brake){
      io.setShooterFeederMode(brake);
    }
    else{
      io.setShooterFeederMode(brake);
    }
  }

    @Override
    public void periodic(){
        io.updateInputs(rollerIOInputs);
        Logger.processInputs(getName(), rollerIOInputs);
    }
}
