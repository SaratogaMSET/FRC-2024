package frc.robot.subsystems.Intake.RollerSubsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.Intake.Roller;

public class RollerIOReal implements RollerIO {
  TalonFX motor;
  DigitalInput IRGate;

  public RollerIOReal() {
    motor = new TalonFX(Roller.MOTOR, "Placeholder");
    IRGate = new DigitalInput(Roller.IR_GATE);
    motor.setNeutralMode(Roller.NEUTRAL_MODE);
    // Set motor output configs for configuring deadband
    MotorOutputConfigs rollerTalonOutputConfigs = new MotorOutputConfigs();
    TalonFXConfiguration rollerTalonConfigs = new TalonFXConfiguration();
    rollerTalonOutputConfigs.DutyCycleNeutralDeadband = Roller.NEUTRAL_VOLTAGE;
    rollerTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0; // TODO: change later
    rollerTalonConfigs.withMotorOutput(rollerTalonOutputConfigs);

    motor.getConfigurator().apply(rollerTalonConfigs);
  }

  @Override
  /** Sets the roller speed */
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }


  @Override
  /** Updates input for the roller speed, and shooter & roller IR */
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocity = motor.get();
    inputs.rollerIR = IRGate.get();
  }
}