package frc.robot.subsystems.Intake.Roller;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.Intake.Roller;

public class RollerIOReal implements RollerIO {
  TalonFX motor;
  DigitalInput IRGate;

  public RollerIOReal() {

    if (Constants.getRobot() == RobotType.ROBOT_2024P){
      motor = new TalonFX(51, "rio");
      IRGate = new DigitalInput(0); // Placeholder
    } else {
      motor = new TalonFX(Intake.Roller.MOTOR, Constants.canbus);
      IRGate = new DigitalInput(Roller.IR_GATE);
    }
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
    SmartDashboard.putNumber("hello please find me", 1);
    motor.setVoltage(voltage);
  }


  @Override
  /** Updates input for the roller speed, and shooter & roller IR */
  public void updateInputs(RollerIOInputs inputs) {
    inputs.velocity = motor.get(); // not velocity is set speed.
    inputs.rollerIR = !IRGate.get();
  }
}