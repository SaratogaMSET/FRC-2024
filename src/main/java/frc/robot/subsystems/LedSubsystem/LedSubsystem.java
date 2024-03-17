package frc.robot.subsystems.LedSubsystem;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Candles;

public class LedSubsystem extends SubsystemBase {
    public CANdle led = new CANdle(Candles.led, Constants.CANBus);
    private int _r = 0;
    private int _g = 0;
    private int _b = 255;
    public enum ColorPositions {
        INTAKE_AMP_MODE,
        INTAKE_GROUND_MODE,
        INTAKE_SPEAKER_MODE
    }

    public LedSubsystem() {
        CANdleConfiguration candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = true;
        candleConfiguration.disableWhenLOS = false;
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.brightnessScalar = 1.0;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        led.configAllSettings(candleConfiguration);
    }
    public Command setColor(int r, int g, int b) {
        return this.runOnce(() -> led.setLEDs(r, g, b));

    }

}
