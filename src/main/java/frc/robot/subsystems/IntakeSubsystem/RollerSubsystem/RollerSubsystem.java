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
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOInputsAutoLogged;

public class RollerSubsystem implements RollerSubsystemIO {
  TalonFX roller;
  DigitalInput enterIrGate = new DigitalInput(Roller.ENTER_IR_GATE);
  DigitalInput exitIrGate = new DigitalInput(Roller.EXIT_IR_GATE);

  public RollerSubsystem() {
    roller = new TalonFX(Roller.ROLLER_MOTOR, "Placeholder");
    enterIrGate = new DigitalInput(Roller.ENTER_IR_GATE);
    exitIrGate = new DigitalInput(Roller.EXIT_IR_GATE);
    roller.setNeutralMode(Roller.ROLLER_NEUTRAL_MODE);
    // Set motor output configs for configuring deadband
    MotorOutputConfigs rollerTalonOutputConfigs = new MotorOutputConfigs();
    TalonFXConfiguration rollerTalonConfigs = new TalonFXConfiguration();
    rollerTalonOutputConfigs.DutyCycleNeutralDeadband = Roller.NEUTRAL_VOLTAGE; // TODO: Tune
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    rollerTalonConfigs.Slot0.kP = 0.0;
    rollerTalonConfigs.Slot0.kI = 0.0;
    rollerTalonConfigs.Slot0.kD = 0.0;
    rollerTalonConfigs.Slot0.kV = 0.0;
    rollerTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0; // change later
    rollerTalonConfigs.withMotorOutput(rollerTalonOutputConfigs);

    roller.getConfigurator().apply(rollerTalonConfigs);
  }

  @Override
  public void roll(double speed) {
    roller.set(speed);
  }

  @Override
  public double getSpeed() {
    return roller.get();
  }
  //   public static int rings = 0;

  @Override
  public boolean acquired() {
    //     rings++;               Make sure to check which way rings are going. If outtaking --->
    // rings--
    return enterIrGate.get();
  }
    
  @Override
  public boolean exited() {
    //   rings--;
    return exitIrGate.get();
  }


  @Override 
  public RollerSubsystemIOInputsAutoLogged updateInputs(){
    var inputs = new RollerSubsystemIOInputsAutoLogged();
    inputs.velocity = getSpeed();
    return inputs;
  }
  

//   @Override
//   public void periodic() {
//     SmartDashboard.putBoolean("Is Object in Roller", acquired());
//     SmartDashboard.putBoolean("Is Object out of Roller", exited());
//   }
}
