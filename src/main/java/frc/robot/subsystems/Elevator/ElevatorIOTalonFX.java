package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Constants.Elevator;

public class ElevatorIOTalonFX implements ElevatorIO {
    public TalonFX leftMotor = new TalonFX(Elevator.CLIMB_LEFT_MOTOR, Constants.canbus);
    public TalonFX rightMotor = new TalonFX(Elevator.CLIMB_RIGHT_MOTOR, Constants.canbus);
    DigitalInput hallEffect = new DigitalInput(Elevator.HALLEFFECT);
    private final StatusSignal<Double> leftPosition = leftMotor.getPosition();
    private final StatusSignal<Double> leftVelocity = leftMotor.getVelocity();
    private final StatusSignal<Double> leftVoltage = leftMotor.getMotorVoltage();
    private final StatusSignal<Double> leftCurrent = leftMotor.getStatorCurrent();
    private final StatusSignal<Double> leftTemp = leftMotor.getDeviceTemp();
    private final StatusSignal<Double> rightPosition = rightMotor.getPosition();
    private final StatusSignal<Double> rightVelocity = rightMotor.getVelocity();
    private final StatusSignal<Double> rightVoltage = rightMotor.getMotorVoltage();
    private final StatusSignal<Double> rightCurrent = rightMotor.getStatorCurrent();
    private final StatusSignal<Double> rightTemp = rightMotor.getDeviceTemp();

    public ElevatorIOTalonFX(){
        var config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 20.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = Elevator.gearing * 2 * Math.PI * Elevator.drumRadiusMeters;
        leftMotor.getConfigurator().apply(config);
        rightMotor.getConfigurator().apply(config);

        leftMotor.setNeutralMode(NeutralModeValue.Brake);
        rightMotor.setNeutralMode(NeutralModeValue.Brake); //SHOULD BE BRAKE
        rightMotor.setInverted(true);

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTemp,
            rightPosition, rightVelocity, rightVoltage, rightCurrent, rightTemp);
        leftMotor.optimizeBusUtilization();
        rightMotor.optimizeBusUtilization();
    }
    @Override
    public void leftSetVoltage(double voltage){
        leftMotor.setVoltage(voltage);
    }
    @Override
    public void rightSetVoltage(double voltage){
        rightMotor.setVoltage(voltage);
    }
    @Override
    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(leftPosition, leftVelocity, leftVoltage, leftCurrent, leftTemp,
            rightPosition, rightVelocity, rightVoltage, rightCurrent, rightTemp);

        inputs.carriagePositionMeters = new double[] {leftPosition.getValueAsDouble(), rightPosition.getValueAsDouble()};
        inputs.secondStagePositionMeters = ((inputs.carriagePositionMeters[0] + inputs.carriagePositionMeters[1])/2) > Units.inchesToMeters(11.375)
            ? ((inputs.carriagePositionMeters[0] + inputs.carriagePositionMeters[1])/2) - Units.inchesToMeters(11.375)
            : 0.0;
        inputs.elevatorVelocityMetersPerSec = new double[]{leftVelocity.getValueAsDouble(), rightVelocity.getValueAsDouble()};
        inputs.elevatorAppliedVolts = new double[]{leftVoltage.getValueAsDouble(), rightVoltage.getValueAsDouble()};
        inputs.elevatorCurrentAmps = new double[] {leftCurrent.getValueAsDouble(), rightCurrent.getValueAsDouble()};
        inputs.hallEffectTriggered = hallEffect.get();
        // inputs.elevatorTempCelsius = new double[] {.getValueAsDouble()};
        inputs.heightLimitTriggered = ((inputs.carriagePositionMeters[0] + inputs.carriagePositionMeters[1])/2.0) >= Elevator.SOFT_LIMIT_HEIGHT;
    }
}
