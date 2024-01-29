package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystem.Roller;
// import com.ctre.phoenix6.motorcontrol.SupplyCurrentLimitConfiguration;   // TODO: Fix current
// limiting

public class RollerSubsystemIOSim implements RollerSubsystemIO {
  TalonFX roller;
  DigitalInput enterIrGate = new DigitalInput(Roller.ENTER_IR_GATE);
  DigitalInput exitIrGate = new DigitalInput(Roller.EXIT_IR_GATE);

  private double velocity = 0.0;

  public RollerSubsystemIOSim() {
    
  }

  public void update() {
    
  }

  public RollerSubsystemIOInputsAutoLogged updateInputs() {
    var inputs = new RollerSubsystemIOInputsAutoLogged();
    inputs.velocity = velocity;
    return inputs;
  }


  public void roll(double speed) {
    roller.set(speed);
  }

  public double getSpeed() {
    return roller.get();
  }
  //   public static int rings = 0;
  public boolean acquired() {
    //     rings++;               Make sure to check which way rings are going. If outtaking --->
    // rings--
    return enterIrGate.get();
  }

  public boolean exited() {
    //   rings--;
    return exitIrGate.get();
  }

  // @Override
  // public void periodic() {
  //   SmartDashboard.putBoolean("Is Object in Roller", acquired()); // ObjectInRoller for 1 ring
  // }
}
