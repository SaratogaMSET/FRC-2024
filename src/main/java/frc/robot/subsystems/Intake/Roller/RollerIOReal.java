package frc.robot.subsystems.Intake.Roller;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.Roller;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.Constants.ShooterFlywheelConstants;

public class RollerIOReal implements RollerIO {
  TalonFX intakeMotor = new TalonFX(Intake.Roller.MOTOR, Constants.CANBus);
  TalonFX shooterMotor = new TalonFX(ShooterFeederConstants.kMotorPort, Constants.CANBus);

  DigitalInput intakeGate = new DigitalInput(Roller.IR_GATE);
  DigitalInput carriageGate = new DigitalInput(Roller.carriageIR);
  DigitalInput shooterGate = new DigitalInput(ShooterFlywheelConstants.kBeamBreakPort);

  public RollerIOReal() {
    intakeMotor.setNeutralMode(Roller.NEUTRAL_MODE);
    // Set motor output configs for configuring deadband
    MotorOutputConfigs rollerTalonOutputConfigs = new MotorOutputConfigs();
    TalonFXConfiguration rollerTalonConfigs = new TalonFXConfiguration();
    rollerTalonOutputConfigs.DutyCycleNeutralDeadband = Roller.NEUTRAL_VOLTAGE;
    rollerTalonConfigs.CurrentLimits.SupplyCurrentLimit = 0; // TODO: change later
    rollerTalonConfigs.withMotorOutput(rollerTalonOutputConfigs);

    intakeMotor.setControl(new StaticBrake());
    intakeMotor.getConfigurator().apply(rollerTalonConfigs);
    intakeMotor.setInverted(true);

    shooterMotor.setControl(new StaticBrake());
    shooterMotor.setInverted(false);
    
    intakeMotor.optimizeBusUtilization();
    shooterMotor.optimizeBusUtilization();
  }

  @Override
  /** Sets the roller speed */
  public void setIntakeFeederVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void setShooterFeederVoltage(double voltage){
    shooterMotor.setVoltage(voltage);
  }

  @Override
  public void setIntakeFeederMode(boolean brake){
    if(brake){
      intakeMotor.setControl(new StaticBrake());
    }
    else{
      intakeMotor.setControl(new CoastOut());
    }
  }

  @Override
  public void setShooterFeederMode(boolean brake){
    if(brake){
      shooterMotor.setControl(new StaticBrake());
    }
    else{
      shooterMotor.setControl(new CoastOut());
    }
  }

  @Override
  /** Updates input for the roller speed, and shooter & roller IR */
  public void updateInputs(RollerIOInputs inputs) {
    inputs.intakeVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
    inputs.shooterVoltage = shooterMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeIR = !intakeGate.get();
    inputs.shooterIR = !shooterGate.get();
    inputs.carriageIR = !carriageGate.get();
  }
}