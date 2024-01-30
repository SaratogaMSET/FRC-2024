package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import org.littletonrobotics.junction.Logger;

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
  private boolean enterIRGate = false;
  private boolean exitIRGate = false;
  private double velocity = 0.0;

  public RollerSubsystemIOSim() {
    
  }

  public void update() {
    Logger.recordOutput("Mechanism2d/" + "Roller", velocity);
  }

  @Override
  public RollerSubsystemIOInputsAutoLogged updateInputs() {
    var inputs = new RollerSubsystemIOInputsAutoLogged();
    inputs.velocity = velocity;
    update();
    return inputs;
  }

  @Override
  public void roll(double speed) {
    velocity = speed;
  }

  @Override
  public double getSpeed() {
    return velocity;
  }

  //   public static int rings = 0;
  public boolean acquired() {
    //     rings++;               Make sure to check which way rings are going. If outtaking --->
    // rings--
    return enterIRGate;
  }

  public boolean exited() {
    //   rings--;
    return exitIRGate;
  }

  // @Override
  // public void simulationPeriodic() {
  //   SmartDashboard.putBoolean("Is Object in Roller", acquired()); // ObjectInRoller for 1 ring
  // }
}
