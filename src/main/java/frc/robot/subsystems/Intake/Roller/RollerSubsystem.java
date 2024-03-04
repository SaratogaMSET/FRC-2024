package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.Logger;

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

    @Override
    public void periodic(){
        io.updateInputs(rollerIOInputs);
        Logger.processInputs(getName(), rollerIOInputs);
    }
}
