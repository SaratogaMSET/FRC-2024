package frc.robot.subsystems.IntakeSubsystem.RollerSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.IntakeSubsystem.Roller;

public class RollerSubsystemIOTalon implements RollerSubsystemIO {
  TalonFX roller;
  DigitalInput enterIrGate = new DigitalInput(Roller.ENTER_IR_GATE);
  DigitalInput exitIrGate = new DigitalInput(Roller.EXIT_IR_GATE);

  public RollerSubsystemIOTalon() {
    roller = new TalonFX(Roller.ROLLER_MOTOR, "Placeholder");
    enterIrGate = new DigitalInput(Roller.ENTER_IR_GATE);
    exitIrGate = new DigitalInput(Roller.EXIT_IR_GATE);
    roller.setNeutralMode(Roller.ROLLER_NEUTRAL_MODE);
    // Set motor output configs for configuring deadband
    MotorOutputConfigs rollerTalonOutputConfigs = new MotorOutputConfigs();
    TalonFXConfiguration rollerTalonConfigs = new TalonFXConfiguration();
    rollerTalonOutputConfigs.DutyCycleNeutralDeadband = Roller.NEUTRAL_VOLTAGE;
    rollerTalonConfigs.Slot0.kP = 0.0;
    rollerTalonConfigs.Slot0.kI = 0.0;
    rollerTalonConfigs.Slot0.kD = 0.0;
    rollerTalonConfigs.Slot0.kV = 0.0;
    rollerTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0; // TODO: change later
    rollerTalonConfigs.withMotorOutput(rollerTalonOutputConfigs);

    roller.getConfigurator().apply(rollerTalonConfigs);
  }

  @Override
  public void roll(double speed) {
    roller.set(speed);
  }

  @Override
  public double getVelocity() {
    return roller.get();
  }

  @Override
  public boolean wristIR() {
    return enterIrGate.get();
  }
    
  @Override
  public boolean shooterIR() {
    return exitIrGate.get();
  }


  @Override
  public void updateInputs(RollerSubsystemIOInputs inputs) {
    inputs.velocity = getVelocity();
    inputs.shooterIR = shooterIR();
    inputs.wristIR = wristIR();
  }
}